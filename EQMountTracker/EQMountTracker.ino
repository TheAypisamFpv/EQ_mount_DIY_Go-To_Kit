/*
  EQMountTracker.ino
  DIY Equatorial Mount Go-To / Tracking Kit - Arduino Mega 2560 Firmware

  Features:
  - Object oriented Axis class using AccelStepper (non-blocking)
  - Sidereal, Solar and Lunar tracking modes with on-board position calculation
  - SolTrack for accurate Sun position (RA/DEC)
  - Moon position using low-precision model + ecliptic->equatorial
  - Full alignment sequence with lead-time compensation and error reporting
  - Serial protocol at 250000 baud for Python GUI
  - Completely non-blocking design

  Required libraries (install via Arduino Library Manager):
    - AccelStepper by Mike McCauley

  Hardware:
    - Elegoo/Elegoo Mega 2560
    - CNC Shield V3 (or compatible) + TMC2209 drivers, STANDALONE mode (MS1/MS2 jumpers, no
      UART wiring currently - see MICROSTEPS below)
    - Two NEMA17 steppers for RA and DEC

  MICROSTEPPING: currently fixed via the MS1/MS2 hardware jumpers on the CNC Shield (no UART -
  see the TODO.md "UART wiring" entry for the researched-but-not-yet-installed UART approach,
  which would allow switching microsteps live in firmware instead of one fixed value). Set
  MICROSTEPS below to match whatever jumpers you actually install; the TMC2209's own standalone
  truth table (NOT the A4988 table silkscreened on the shield - they differ) is:
      MS1=LOW,MS2=LOW -> 8 microsteps      MS1=HIGH,MS2=LOW -> 32 microsteps
      MS1=LOW,MS2=HIGH -> 64 microsteps    MS1=HIGH,MS2=HIGH -> 16 microsteps
  No jumpers installed = MS1=MS2=LOW = 8 microsteps, which is what MICROSTEPS is set to below.

  Adjust pins, gear ratios, speeds and location (real GPS from GUI) below.

  Folder must be named EQMountTracker for Arduino IDE to recognize the sketch properly.
*/

#include <Arduino.h>
#include "Axis.h"
#include <math.h>   // for sin, cos, asin, floor, fmod, etc. used by SolTrack and moon calculations
#include <string.h> // for strtok, strncpy, strstr
#include <stdio.h>  // for sscanf, snprintf if needed
#include <stdlib.h> // for atof
#include <EEPROM.h> // for EEPROM storage

// ============================================================
// CONFIGURATION - ADJUST FOR YOUR HARDWARE
// ============================================================

// Serial
const long SERIAL_BAUD = 250000;

// CNC Shield V3 pin mapping (X=RA, Z=DEC - the Y socket is unused). Enable is usually shared
// (pin 8). On Mega these pins are compatible with standard CNC Shield.
const uint8_t PIN_STEP_RA   = 2;
const uint8_t PIN_DIR_RA    = 5;
const uint8_t PIN_STEP_DEC  = 4;
const uint8_t PIN_DIR_DEC   = 7;
const uint8_t PIN_ENABLE    = 8;   // Shared enable pin for both drivers (active LOW)

// Gear ratios: TOTAL motor steps (including microstepping) per DEGREE on the sky.
// Your gearing: 130:1 mechanical (130 motor revs per axis rev)
// steps_per_deg = 200 * usteps * 130 / 360
//
// IMPORTANT: this does NOT change what the driver actually does - no UART is wired, so real
// microstepping is fixed by hardware (MS1/MS2 pin state, no jumpers currently installed) at
// whatever that really is; this constant is only the firmware's ASSUMPTION, used for its own
// speed/step-rate math. Setting it to something other than the true real value doesn't change
// physical behavior - it just makes commanded/displayed speed inaccurate relative to real speed
// (see the MICROSTEPPING note in the file header comment for the full explanation).
//
// Currently 8 - this is the best-confirmed value so far: MICROSTEPS=8 with
// MAX_SLEW_SPEED_DEG_S=2.0 was tested on real hardware and ran WITHOUT vibration. There was
// separate (weaker) evidence from a GUI-displayed-vs-real speed comparison suggesting the true
// value might actually be 16, but that was a rough "less than 1 deg/s" eyeball estimate, not a
// precise measurement - 8 remains the working assumption until something more precise overrides
// it. To get a precise measurement and settle this for good:
//   1. Command a slew of a known, large angle (e.g. exactly 90 deg) at a known commanded speed.
//   2. Time it with a stopwatch from first movement to stop.
//   3. real_deg_per_sec = 90 / measured_seconds.
//   4. real_microsteps = MICROSTEPS * (commanded_deg_per_sec / real_deg_per_sec).
//      If that comes out ~8, this assumption is confirmed. If it comes out ~16, ~32, etc.,
//      update MICROSTEPS to that value instead - each of those is a normal, valid TMC2209
//      standalone value, so whichever it lands on is plausible.
// A multimeter reading directly on the MS1/MS2 pins (~0V = LOW, ~5V = HIGH) against the
// standalone truth table below is the other way to confirm this with certainty, independent of
// any timing measurement:
//   MS1=LOW,MS2=LOW->8   MS1=HIGH,MS2=LOW->32   MS1=LOW,MS2=HIGH->64   MS1=HIGH,MS2=HIGH->16
const int MICROSTEPS = 8;
// switchMicrosteps() is currently a no-op (see its definition below), but the alignment state
// machine still calls it at each slew/tracking phase transition with these two names - kept as
// aliases of MICROSTEPS so those call sites don't all need editing right now. Once UART
// switching is wired back in, give these their own distinct values again (e.g. 1 for fast
// slews, 256 for fine tracking) and restore the real switching logic in switchMicrosteps().
const int TRACKING_MICROSTEPS = MICROSTEPS;
const int SLEW_MICROSTEPS     = MICROSTEPS;

float getGearRatio(int microsteps) {
  return 200.0f * microsteps * 130.0f / 360.0f;
}

// Forward declarations
void switchMicrosteps(int newMicrosteps);

// Motion limits (sky degrees / second)
// Dropped from 5.0 to 2.0 deg/s: even at the original conservative 5.0/5.0s (1.0 deg/s^2)
// values, slews were still noisy/vibrating - the math points at motor mechanical resonance,
// not the acceleration curve. At 8 microsteps with 130:1 gearing, axis speed converts to motor
// shaft speed as (deg/s / 360) * 130 * 60 RPM: 5 deg/s -> ~108 RPM, squarely inside the ~60-240
// RPM band where many NEMA17 motors have a resonance peak. 2 deg/s -> ~43 RPM, clearly below
// where that typically starts. This is a diagnostic/interim value - a telescope tracker doesn't
// need fast slews the way a 3D printer needs print speed, so erring slow is low-cost here.
// Once MS1/MS2 jumpers are installed for finer microstepping, the same axis speed maps to a
// much higher step frequency (less likely to hit the resonance band at all), and this can
// likely come back up - re-tune once that's done.
const float MAX_SLEW_SPEED_DEG_S   = 2.0f;   // Max speed for ALL non-tracking movements (sky °/s at axis)

// Position update broadcast interval (can be changed from Python GUI)
unsigned long positionUpdateIntervalMs = 20;   // default 50 Hz (GUI can override)
const float TRACKING_MAX_SPEED     = 0.1f;   // Max speed used during tracking updates
const float SLEW_RAMP_TIME_S       = 5.0f;   // Desired time to accelerate from 0->max (and decelerate max->0). Same for accel and decel.
const float SLEW_ACCEL_DEG_S2      = MAX_SLEW_SPEED_DEG_S / SLEW_RAMP_TIME_S;  // 0.4 °/s² (2.0/5.0). Used for both acceleration and deceleration.

// Alignment parameters
const float ACCEPTABLE_ERROR_DEG   = 0.015f; // When alignment error is good enough to start tracking
const float LEAD_COEFFICIENT       = 2.0f;   // Extra margin (seconds worth) for lead time calc to ensure we arrive early enough
const unsigned long ERROR_UPDATE_MS = 100;   // 10 Hz error reports during WAIT phase

// Default observer location (used by SolTrack for topocentric corrections where relevant).
// Placeholder only - Royal Observatory Greenwich (public astronomical landmark, home of the
// Prime Meridian), deliberately NOT a real/private address. Overwritten by CMD,SET_LOCATION
// from the GUI (which then persists it to EEPROM via CMD,SAVE_LOCATION_EEPROM) - update via
// that, not by editing this default.
// (Ground safeguard removed; these are still used for sun/moon calculations.)
float OBS_LAT_DEG = 51.4769f;   // Positive north
float OBS_LON_DEG = -0.0005f;   // Positive east

// EEPROM storage for location persistence across power cycles
#define EEPROM_LAT_ADDR 0
#define EEPROM_LON_ADDR 4
#define EEPROM_MAGIC_ADDR 8
#define EEPROM_MAGIC_VALUE 0xA5

// Software version
// Changelog:
//   1.3.0 - Prior baseline.
//   1.4.0 - Fixed START_TRACKING,SKIP_DEC_RESET being silently ignored: the command parser
//           truncated the action name at the first comma, so the SKIP_DEC_RESET suffix was
//           never seen and every Start Tracking ran the full DEC-reset sequence. Merged
//           startAlignmentSequence()/startAlignmentSequenceSkipDecReset() into one function
//           taking a skipDecReset flag to avoid this kind of divergence in the future.
//   1.5.0 - Fixed SYNC_OFFSET (the RA/DEC fine-tune nudge) applying its delta to BOTH the
//           calibration (cal_sky_ra/dec) AND the target (targetRA/DEC, siderealTarget*,
//           sync*) by the same amount. Since mount position is always computed as
//           cal_mount + (target - cal_sky) + drift, that cancelled out: the mount never
//           physically moved, while cal_sky_ra/dec -- the single mount<->sky reference
//           shared by every mode -- permanently drifted, silently mis-pointing every later
//           sun/moon/star target too. Now the nudge only adjusts cal_mount_ra/dec, so it
//           actually re-points the mount and leaves other targets unaffected.
//   1.6.0 - Fixed the main alignment sequence (ALIGN_RESETTING_DEC -> ALIGN_RA transition):
//           (1) it was commanding DEC to slew to its target at the same time as RA, instead
//           of only after RA finished (DEC is supposed to stay parked at its reset position
//           until ALIGN_WAIT_RA hands off to ALIGN_DEC). (2) the sidereal RA lead-time formula
//           added `moveTime` (a value in SECONDS) directly onto a sky position in DEGREES, and
//           never referenced the actual target RA at all - harmless for the sub-second
//           corrections during normal continuous tracking, but for any real slew to a new,
//           distant target (e.g. re-targeting from Vega to Tau Ceti) it produced a nonsensical
//           offset unrelated to the target.
//   1.6.1 - 1.6.0 also added a "shortest rotation" (<=180 deg) rule for RA slew targets, on the
//           assumption that a big rotation was itself the cable-wrap risk. That was backwards
//           for this mount: the RA cable can't tolerate crossing the RA=0/360 seam AT ALL, in
//           either direction, regardless of resulting distance - so "shortest path" is exactly
//           wrong whenever the short way happens to cross that seam (e.g. 350 deg -> 5 deg must
//           go the LONG way, 350->300->...->5, not the 15 deg way through 0). Reverted to the
//           direct/unwrapped sky-RA delta (target - current, both already in [0,360)), which by
//           construction moves straight between the two values on the number line and can never
//           cross the seam, however far that ends up being. Removed shortestDeltaDeg().
//   1.7.0 - The SKIP_DEC_RESET path (startAlignmentSequence's skipDecReset=true branch, used
//           whenever the new target is close to the current position - e.g. stopping and
//           restarting tracking in SOLAR/LUNAR mode, where the sun/moon barely moved) had its
//           own separate ALIGN_SLEWING_TO_TARGET state that commanded RA and DEC to slew
//           CONCURRENTLY, unlike the normal path's strictly sequential RA-then-DEC. Since the
//           GUI picks this path automatically whenever the target is within 5 deg - which for
//           SOLAR/LUNAR is true almost every time you restart tracking shortly after stopping
//           - both axes moving at once was the common case for those modes, not a rare one.
//           Concurrent moves with independent, distance-dependent AccelStepper ramp profiles
//           also explains the acceleration looking "not respected": whichever axis had the
//           shorter distance would ramp up only briefly (or not reach a real ramp at all) and
//           stop, while the other kept accelerating, which reads as an abrupt/instant move.
//           ALIGN_SLEWING_TO_TARGET is removed entirely. skipDecReset now only changes what
//           the DEC-reset step does (hold DEC in place instead of slewing it out to 0 and
//           back), then falls through into the exact same sequential RA-then-DEC pipeline the
//           normal path uses - one axis moving at a time, every time, in every mode.
//   1.7.1 - CMD,MODE only recomputed targetRA/DEC for the new mode if trackingActive was
//           already true, but sendPositionUpdate() reports targetRA/DEC in every POS update
//           regardless of tracking state - so switching mode while stopped left POS reporting
//           the PREVIOUS mode's stale target until tracking was (re)started. A GUI client's
//           "is the new target close to the current position?" check comparing against that
//           stale value right after a mode switch could wrongly treat a brand new Sun/Moon
//           target as "close" because it was still comparing against the old mode's target.
//           Now recomputed unconditionally on any mode change.
//   1.7.2 - All RA/DEC/calibration/location values sent over serial (POS updates, STATUS
//           lines, DEBUG dumps) were printed with 4 decimal places; bumped to 6 to match the
//           GUI's display/logging precision and give the sub-pixel-level tracking-error
//           resolution the camera sensor can actually distinguish (~0.0003 deg/pixel) room to
//           be represented rather than getting rounded away at the wire format.
//   1.7.3 - Replaced the default OBS_LAT_DEG/OBS_LON_DEG placeholder with Royal Observatory
//           Greenwich (public astronomical landmark) instead of an arbitrary round-number
//           location, ahead of publishing this project's source - no functional change (the
//           GUI always overwrites this via CMD,SET_LOCATION before it matters).
//   1.8.0 - Added real TMC2209 UART microstepping control (TMCStepper library). Previously
//           switchMicrosteps() only updated the firmware's own steps-per-degree assumption;
//           the actual driver microstepping was whatever the hardware MS pins/jumpers happened
//           to be set to, with no way for the firmware to detect or change it - if those
//           didn't match SLEW_MICROSTEPS/TRACKING_MICROSTEPS (likely on a stock CNC Shield V3,
//           which doesn't expose a fixed "1" full-step option in typical TMC2209 standalone-
//           mode jumper combinations), commanded speed and real physical speed diverged by
//           that mismatch factor - which is why large slews were topping out around 1.7 deg/s
//           instead of the commanded 5 deg/s regardless of distance. Requires wiring PDN_UART
//           from each module to its own dedicated Mega hardware UART (see the file header's
//           "TMC UART WIRING" section) - MOTOR_RMS_CURRENT_MA and R_SENSE MUST be set to match
//           your actual motor/module before flashing. Also bumped MAX_SLEW_SPEED_DEG_S
//           5.0->6.5 and shortened SLEW_RAMP_TIME_S 5.0->3.5s now that real achieved speed can
//           actually match commanded speed - re-tune both if you see skipped steps/stalls.
//   1.8.1 - Reverted 1.8.0's UART microstepping control for now (TMCStepper, driverRA/DEC, the
//           setup() init block, and TMC_RA/DEC_SERIAL) - the physical UART wiring isn't
//           installed yet (driver modules plug straight into the CNC Shield socket as one
//           unit, no accessible tap point without soldering onto the pin legs directly, which
//           hasn't been done). Back to a single fixed MICROSTEPS constant (8, matching no MS1/
//           MS2 jumpers installed), set via hardware jumpers same as before 1.8.0.
//           switchMicrosteps() is now a no-op stub; TRACKING_MICROSTEPS/SLEW_MICROSTEPS are
//           kept as aliases of MICROSTEPS so the alignment sequence's existing calls to it
//           don't need editing. The researched UART wiring plan (confirmed GERUI TMC2209 V2.0
//           pinout) is preserved in TODO.md for whenever the physical wiring gets done.
//   1.8.2 - Reverted the MAX_SLEW_SPEED_DEG_S/SLEW_RAMP_TIME_S bump from 1.8.0 (6.5 deg/s,
//           3.5s ramp) back to the original 5.0/5.0s - that bump was only justified by "UART
//           will make real speed match commanded speed", which no longer holds now that UART
//           is reverted (1.8.1, back to a fixed coarse 8-microstep setup). Coarse microstepping
//           is also more prone to hitting motor mechanical resonance at a given speed (fewer,
//           bigger steps per degree means a lower step frequency for the same angular speed),
//           and a steeper ramp spends more time accelerating through whatever resonance band
//           exists - matches reported "extremely noisy/vibrates a lot" during high-speed
//           movement. Re-tune once finer microstepping (MS1/MS2 jumpers) is installed.
//   1.8.3 - DEC now wired to the CNC Shield's Z axis socket (STEP=pin4, DIR=pin7) instead of Y
//           (STEP=pin3, DIR=pin6) - RA stays on X. Y socket is unused.
//   1.8.4 - 1.8.2's revert back to 5.0 deg/s (from 6.5) didn't fix reported vibration/noise -
//           confirmed on-hardware to still occur at slew speed even at the original
//           conservative acceleration values. Dropped MAX_SLEW_SPEED_DEG_S to 2.0 deg/s: at 8
//           microsteps with 130:1 gearing, axis speed converts to motor shaft RPM as
//           (deg/s / 360) * 130 * 60 - 5 deg/s was ~108 RPM, squarely inside the ~60-240 RPM
//           band where many NEMA17 motors have a resonance peak, which fits "noisy specifically
//           during fast movement" better than an acceleration-curve explanation. 2 deg/s is
//           ~43 RPM, clearly under where that typically starts. Diagnostic/interim value - a
//           telescope tracker doesn't need fast slews the way a 3D printer needs print speed,
//           so erring slow is low-cost. Revisit once MS1/MS2 jumpers give finer microstepping
//           (same axis speed then maps to a much higher, less resonance-prone step frequency).
//   1.8.5 - Reverted MICROSTEPS from 8 back to 1 (see its comment) at user request, to
//           reproduce the quiet behavior seen before this constant existed. NOTE: real evidence
//           (GUI-reported vs actual measured slew speed, ~2x off) suggests the real hardware
//           microstepping is actually 16, not 8 as previously assumed - MICROSTEPS=1 doesn't
//           change real driver behavior (no UART wired), it just deliberately understates the
//           gear ratio in the firmware's own math, dividing real achieved speed down by
//           whatever the true mismatch ratio is (~16x) versus what's commanded/displayed. This
//           reproduces the old accidental quietness (real speed ends up a small fraction of
//           commanded) but does NOT fix speed-reporting accuracy or resolve the underlying
//           coarse-step issue - both still need the real microstep value confirmed (multimeter
//           on MS1/MS2, or a precise timed-slew measurement) and, ultimately, jumpers installed.
//   1.8.6 - Reverted MICROSTEPS back to 8 (undoing 1.8.5) - MICROSTEPS=1 was never a real fix,
//           just a "lie to the math" that made real motion a fraction of commanded speed, which
//           made slews impractically slow rather than fixing anything. 8 is the last value that
//           was actually confirmed vibration-free on real hardware (at MAX_SLEW_SPEED_DEG_S=2.0,
//           see 1.8.4) and is the honest software assumption matching "no jumpers installed"
//           (TMC2209 standalone default MS1=LOW,MS2=LOW -> 8). The real hardware value is still
//           not definitively confirmed - see the expanded comment above MICROSTEPS for both a
//           precise timed-slew measurement procedure and the MS1/MS2 multimeter check to settle
//           it for good.
//   1.8.7 - Found the REAL cause of the vibration/twitch that 1.8.4-1.8.6 were only working
//           around by slowing everything down: sendPositionUpdate()'s message is ~190 bytes, but
//           the Mega's hardware Serial TX buffer is only 64 bytes - Serial.print() blocks once
//           it fills, waiting for the UART to drain, which stalls the whole loop() for several
//           ms at 250000 baud. axisRA.update()/axisDEC.update() (non-blocking step generators)
//           don't get serviced again until the blocked print call returns, so every position
//           broadcast was directly stalling step timing - worse at higher POS_UPDATE rates
//           (more frequent stalls), confirmed by user observation: 50Hz was rough, 10Hz much
//           smoother, 5Hz gave a distinct single twitch synced exactly to each broadcast.
//           Fixed by interleaving axisRA.update()/axisDEC.update() calls between print segments
//           in sendPositionUpdate() (breaks one long block into several short serviced ones) and
//           trimming float precision from 6 to 4 decimals (still exceeds real single-microstep
//           resolution, just shortens the message). MICROSTEPS/MAX_SLEW_SPEED_DEG_S from 1.8.4
//           are left as-is for now (still reasonable values) but this was the actual root cause -
//           worth re-testing higher slew speeds now that this stall is fixed.
//   1.8.8 - Shrunk the POS message further at the user's request to minimize bytes sent: switched
//           from labeled KEY:value pairs to a fixed-order positional CSV (labels like
//           ",TARGET_RA:" were pure overhead - the GUI parser can count commas instead), dropped
//           ERR_RA/ERR_DEC from the wire entirely (GUI now reconstructs them locally as
//           abs(sky-target), same value, zero wire cost), and MODE is now a single character
//           (S/O/L) instead of the full word. Message is now ~55-60 bytes, comfortably under the
//           Mega's 64-byte Serial TX buffer that was causing the blocking stalls fixed in 1.8.7 -
//           at this size most updates should avoid that blocking path entirely rather than just
//           recovering from it quickly. tracker_gui.py's POS parser updated to match this exact
//           field order: POS,skyRA,skyDEC,targetRA,targetDEC,mountRA,mountDEC,mode,tracking.
//   1.8.9 - Shrinking the POS message (1.8.8) didn't fully fix the broadcast-synced twitch -
//           user confirmed it's still audible even at ~55-60 bytes, exactly on each broadcast.
//           Root cause: Serial.print(float, 4) itself is CPU-bound (float-to-ASCII conversion
//           on an AVR with no hardware FPU, ~100-300us per call) independent of message size or
//           wire format - six of them back to back in one sendPositionUpdate() call adds up to
//           roughly the same order of magnitude as the ~865us step interval AccelStepper needs
//           during a slew, and since AccelStepper is polled (not interrupt-driven), that stall
//           delays step generation directly. Fixed by replacing sendPositionUpdate() with
//           updatePositionBroadcast(), a small state machine called every loop() iteration that
//           emits one or two fields per call instead of the whole message in one shot - caps the
//           worst-case single stall this adds to any one loop() iteration at roughly one float
//           conversion instead of six. Doesn't fully eliminate the underlying issue (see the
//           TODO.md entry on moving step generation to a hardware timer interrupt for the real
//           fix), but should reduce it substantially.
//   1.8.10 - Build fix: 1.8.9 renamed sendPositionUpdate() to the incremental
//            updatePositionBroadcast() but missed 5 other call sites that need a synchronous,
//            immediate, complete send (boot, SET_TIME, SET_LOCATION, offset-applied, debug
//            snapshot restore) rather than one spread across loop() iterations - those aren't
//            time-critical/mid-slew events, so blocking briefly there is fine. Added
//            sendPositionUpdateNow() (same field layout, single-shot like the old function) for
//            those call sites; updatePositionBroadcast() remains the periodic, non-blocking one
//            used from loop().
#define FIRMWARE_VERSION "1.8.10"

// Sidereal rate (deg/sec on sky for RA axis). Approx 15.041 arcsec/s.
const float SIDEREAL_RATE_DEG_S = 0.004178f;

// ============================================================
// SOLTRACK (Sun position) - Bundled lightweight library
// Adapted from Marc van der Sluys SolTrack-Arduino (LGPL)
// https://github.com/MarcvdSluys/SolTrack-Arduino
// ============================================================

#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)
#define R2D 57.2957795130823208768
#define R2H 3.81971863420548805845

struct STTime {
  int year, month, day, hour, minute;
  double second;
};

struct STLocation {
  double longitude, latitude;
  double sinLat, cosLat;
  double pressure, temperature;
};

struct STPosition {
  double julianDay, tJD, tJC, tJC2, UT;
  double longitude, distance;
  double obliquity, cosObliquity, nutationLon;
  double rightAscension, declination, hourAngle, agst;
  double altitude, altitudeRefract, azimuthRefract;
  double hourAngleRefract, declinationRefract;
};

// Moon position return type (used by lunar tracking)
struct MoonPos {
  double raDeg;
  double decDeg;
};

// Forward declarations
MoonPos computeMoonPosition(const STTime &t, double obsLatDeg, double obsLonDeg);
void startAlignmentSequence(bool skipDecReset = false);
void stopAll();
void stopAlignment();
void runAlignmentSequence();
void updateTrackingModes();
void saveLocationToEEPROM();
void loadLocationFromEEPROM();
void queryLocation();
void readSerialCommands();
void sendStatus(const char* action, const char* axis, float error);
void updatePositionBroadcast();
void sendPositionUpdateNow();
float estimateMovementTime(float deltaDeg, float speedDegS, float accelDegS2);
void dumpDebugState(const char* trigger);
void parseAndExecuteCommand(char* cmd);

double computeJulianDay(int year, int month, int day, int hour, int minute, double second);
void computeLongitude(int computeDistance, struct STPosition *position);
void convertEclipticToEquatorial(double longitude, double cosObliquity, double *rightAscension, double *declination);
void convertEquatorialToHorizontal(struct STLocation location, struct STPosition *position);
void eq2horiz(double sinLat, double cosLat, double longitude, double rightAscension, double declination, double agst, double *azimuth, double *sinAlt);
void convertHorizontalToEquatorial(double sinLat, double cosLat, double azimuth, double altitude, double *hourAngle, double *declination);
void setNorthToZero(double *azimuth, double *hourAngle, int computeRefrEquatorial);
void convertRadiansToDegrees(double *longitude, double *rightAscension, double *declination,
                             double *altitude, double *azimuthRefract, double *altitudeRefract,
                             double *hourAngle, double *declinationRefract, int computeRefrEquatorial);
double STatan2(double y, double x);

void SolTrack(struct STTime time, struct STLocation location, struct STPosition *position,
              int useDegrees, int useNorthEqualsZero, int computeRefrEquatorial, int computeDistance) {
  struct STLocation llocation = location;
  if (useDegrees) {
    llocation.longitude /= R2D;
    llocation.latitude  /= R2D;
  }
  llocation.sinLat = sin(llocation.latitude);
  llocation.cosLat = sqrt(1.0 - llocation.sinLat * llocation.sinLat);

  position->julianDay = computeJulianDay(time.year, time.month, time.day, time.hour, time.minute, time.second);
  position->UT = time.hour + (double)time.minute / 60.0 + (double)time.second / 3600.0;

  position->tJD  = position->julianDay;
  position->tJC  = position->tJD / 36525.0;
  position->tJC2 = position->tJC * position->tJC;

  computeLongitude(computeDistance, position);
  convertEclipticToEquatorial(position->longitude, position->cosObliquity, &position->rightAscension, &position->declination);
  convertEquatorialToHorizontal(llocation, position);

  if (computeRefrEquatorial) {
    convertHorizontalToEquatorial(llocation.sinLat, llocation.cosLat, position->azimuthRefract,
                                  position->altitudeRefract, &position->hourAngleRefract, &position->declinationRefract);
  }
  if (useNorthEqualsZero) {
    setNorthToZero(&position->azimuthRefract, &position->hourAngleRefract, computeRefrEquatorial);
  }
  if (useDegrees) {
    convertRadiansToDegrees(&position->longitude, &position->rightAscension, &position->declination,
                            &position->altitude, &position->azimuthRefract, &position->altitudeRefract,
                            &position->hourAngleRefract, &position->declinationRefract, computeRefrEquatorial);
  }
}

double computeJulianDay(int year, int month, int day, int hour, int minute, double second) {
  if (month <= 2) {
    year -= 1;
    month += 12;
  }
  int tmp1 = (int)floor(year / 100.0);
  int tmp2 = 2 - tmp1 + (int)floor(tmp1 / 4.0);
  double dDay = day + hour / 24.0 + minute / 1440.0 + second / 86400.0;
  double JD = floor(365.250 * (year - 2000)) - 50.5 + floor(30.60010 * (month + 1)) + dDay + tmp2;
  return JD;
}

void computeLongitude(int computeDistance, struct STPosition *position) {
  double l0 = fmod(4.895063168 + 628.331966786 * position->tJC + 5.291838e-6 * position->tJC2, TWO_PI);
  double m  = fmod(6.240060141 + 628.301955152 * position->tJC - 2.682571e-6 * position->tJC2, TWO_PI);

  double c = fmod((3.34161088e-2 - 8.40725e-5 * position->tJC - 2.443e-7 * position->tJC2) * sin(m) +
                  (3.489437e-4 - 1.76278e-6 * position->tJC) * sin(2 * m), TWO_PI);
  double odot = l0 + c;

  double omg  = fmod(2.1824390725 - 33.7570464271 * position->tJC + 3.622256e-5 * position->tJC2, TWO_PI);
  double dpsi = -8.338601e-5 * sin(omg);
  double dist = 1.0000010178;
  if (computeDistance) {
    double ecc = 0.016708634 - 0.000042037 * position->tJC - 0.0000001267 * position->tJC2;
    double nu = m + c;
    dist = dist * (1.0 - ecc * ecc) / (1.0 + ecc * cos(nu));
  }
  double aber = -9.93087e-5 / dist;

  double eps0 = 0.409092804222 - (2.26965525e-4 * position->tJC + 2.86e-9 * position->tJC2);
  double deps = 4.4615e-5 * cos(omg);

  position->longitude   = fmod(odot + aber + dpsi, TWO_PI);
  position->distance    = dist;
  position->obliquity   = eps0 + deps;
  position->cosObliquity = cos(position->obliquity);
  position->nutationLon = dpsi;
}

void convertEclipticToEquatorial(double longitude, double cosObliquity, double *rightAscension, double *declination) {
  double sinLon = sin(longitude);
  double sinObl = sqrt(1.0 - cosObliquity * cosObliquity);
  *rightAscension = STatan2(cosObliquity * sinLon, cos(longitude));
  *declination    = asin(sinObl * sinLon);
}

void convertEquatorialToHorizontal(struct STLocation location, struct STPosition *position) {
  double gmst = 1.75336856 + fmod(0.017202791805 * position->tJD, TWO_PI) + 6.77e-6 * position->tJC2 + position->UT / R2H;
  position->agst = fmod(gmst + position->nutationLon * position->cosObliquity, TWO_PI);

  double sinAlt = 0.0;
  eq2horiz(location.sinLat, location.cosLat, location.longitude, position->rightAscension, position->declination,
           position->agst, &position->azimuthRefract, &sinAlt);

  double alt = asin(sinAlt);
  double cosAlt = sqrt(1.0 - sinAlt * sinAlt);
  alt -= 4.2635e-5 * cosAlt;   // parallax
  position->altitude = alt;

  double dalt = 2.967e-4 / tan(alt + 3.1376e-3 / (alt + 8.92e-2));
  dalt *= location.pressure / 101.0 * 283.0 / location.temperature;
  alt += dalt;
  position->altitudeRefract = alt;
}

void eq2horiz(double sinLat, double cosLat, double longitude, double rightAscension, double declination, double agst,
              double *azimuth, double *sinAlt) {
  double ha = agst + longitude - rightAscension;
  double sinHa = sin(ha);
  double cosHa = cos(ha);
  double sinDec = sin(declination);
  double cosDec = sqrt(1.0 - sinDec * sinDec);
  double tanDec = sinDec / cosDec;
  *azimuth = STatan2(sinHa, cosHa * sinLat - tanDec * cosLat);
  *sinAlt  = sinLat * sinDec + cosLat * cosDec * cosHa;
}

void convertHorizontalToEquatorial(double sinLat, double cosLat, double azimuth, double altitude,
                                   double *hourAngle, double *declination) {
  double cosAz = cos(azimuth);
  double sinAz = sin(azimuth);
  double sinAlt = sin(altitude);
  double cosAlt = sqrt(1.0 - sinAlt * sinAlt);
  double tanAlt = sinAlt / cosAlt;
  *hourAngle   = STatan2(sinAz, cosAz * sinLat + tanAlt * cosLat);
  *declination = asin(sinLat * sinAlt - cosLat * cosAlt * cosAz);
}

void setNorthToZero(double *azimuth, double *hourAngle, int computeRefrEquatorial) {
  *azimuth = *azimuth + PI;
  if (*azimuth > TWO_PI) *azimuth -= TWO_PI;
  if (computeRefrEquatorial) {
    *hourAngle = *hourAngle + PI;
    if (*hourAngle > TWO_PI) *hourAngle -= TWO_PI;
  }
}

void convertRadiansToDegrees(double *longitude, double *rightAscension, double *declination,
                             double *altitude, double *azimuthRefract, double *altitudeRefract,
                             double *hourAngle, double *declinationRefract, int computeRefrEquatorial) {
  *longitude *= R2D;
  *rightAscension *= R2D;
  *declination *= R2D;
  *altitude *= R2D;
  *azimuthRefract *= R2D;
  *altitudeRefract *= R2D;
  if (computeRefrEquatorial) {
    *hourAngle *= R2D;
    *declinationRefract *= R2D;
  }
}

double STatan2(double y, double x) {
  if (x > 0.0) return atan(y / x);
  else if (x < 0.0) {
    if (y >= 0.0) return atan(y / x) + PI;
    else return atan(y / x) - PI;
  } else {
    if (y > 0.0) return PI / 2.0;
    else if (y < 0.0) return -PI / 2.0;
    else return 0.0;
  }
}

// ============================================================
// MOON POSITION (low precision + conversion)
// Based on MoonPhase concepts + simple conversion. Sufficient for DIY tracking.
// ============================================================

// Full Julian Date (needed for Moon calculations - different from SolTrack's internal tJD)
double julianDateFull(int year, int month, int day, int hour, int minute, double second) {
  if (month <= 2) {
    year -= 1;
    month += 12;
  }
  int A = year / 100;
  int B = 2 - A + (A / 4);
  double JD = floor(365.25 * (year + 4716)) + floor(30.6001 * (month + 1)) + day + B - 1524.5;
  JD += (hour + (minute / 60.0) + (second / 3600.0)) / 24.0;
  return JD;
}

MoonPos computeMoonPosition(const STTime &t, double obsLatDeg, double obsLonDeg) {
  // Use full Julian Date for accurate lunar ephemeris
  double jd = julianDateFull(t.year, t.month, t.day, t.hour, t.minute, t.second);

  // Use MoonPhase style calculations for ecliptic lon/lat
  const double MOON_SYNODIC_PERIOD   = 29.530588853;
  const double MOON_SYNODIC_OFFSET   = 2451550.26;
  const double MOON_LONGITUDE_PERIOD = 27.321582241;
  const double MOON_LONGITUDE_OFFSET = 2451555.8;
  const double MOON_DISTANCE_PERIOD  = 27.55454988;
  const double MOON_DISTANCE_OFFSET  = 2451562.2;
  const double MOON_LATITUDE_PERIOD  = 27.212220817;
  const double MOON_LATITUDE_OFFSET  = 2451565.2;

  double phase = (jd - MOON_SYNODIC_OFFSET) / MOON_SYNODIC_PERIOD;
  phase -= floor(phase);

  double longPhase = (jd - MOON_LONGITUDE_OFFSET) / MOON_LONGITUDE_PERIOD;
  longPhase -= floor(longPhase);
  double longitude = 360.0 * longPhase
                   + 6.3 * sin(2 * PI * ((jd - MOON_DISTANCE_OFFSET) / MOON_DISTANCE_PERIOD))
                   + 1.3 * sin(2 * 2 * PI * phase - 2 * PI * ((jd - MOON_DISTANCE_OFFSET) / MOON_DISTANCE_PERIOD))
                   + 0.7 * sin(2 * 2 * PI * phase);
  if (longitude > 360) longitude -= 360;
  longitude *= PI / 180.0;  // to rad

  double latPhase = (jd - MOON_LATITUDE_OFFSET) / MOON_LATITUDE_PERIOD;
  latPhase -= floor(latPhase);
  double latitude = 5.1 * sin(2 * PI * latPhase) * PI / 180.0; // rad

  // Obliquity approx (same as SolTrack mean)
  double tJC = (jd / 36525.0);
  double eps = 0.409092804222 - (2.26965525e-4 * tJC);

  // Ecliptic to equatorial (full with latitude)
  double sinLon = sin(longitude);
  double cosLon = cos(longitude);
  double sinLat = sin(latitude);
  double cosLat = cos(latitude);
  double sinEps = sin(eps);
  double cosEps = cos(eps);

  double ra  = STatan2( cosLat * sinLon * cosEps - sinLat * sinEps , cosLat * cosLon );
  double dec = asin( cosLat * sinLon * sinEps + sinLat * cosEps );

  MoonPos pos;
  pos.raDeg  = ra * R2D;
  pos.decDeg = dec * R2D;

  // Range RA 0-360
  if (pos.raDeg < 0) pos.raDeg += 360.0;
  return pos;
}

// Calibration from sync: maps mount physical angle to sky at a point in time
// Declared here so the helper functions below can see them (C++ declaration order).
bool isCalibrated = false;
float cal_mount_ra = 0.0;
float cal_sky_ra = 0.0;
float cal_mount_dec = 0.0;
float cal_sky_dec = 0.0;
unsigned long cal_time_ms = 0;

// Debug flag - when true, dumps a lot of internal state variables periodically + on key events.
// Enable from GUI with: CMD,DEBUG,ON   (OFF to disable, DUMP for one-shot)
bool debugVerbose = false;
unsigned long lastDebugDumpMs = 0;

// Compute sky position from mount physical angle + calibration + Earth rotation drift
float computeSkyRA(float mount_angle, unsigned long t_ms) {
  if (!isCalibrated) return mount_angle;
  float delta_m = mount_angle - cal_mount_ra;
  float dt = (t_ms - cal_time_ms) / 1000.0;
  float drift = SIDEREAL_RATE_DEG_S * dt;
  float s = cal_sky_ra + delta_m - drift;
  s = fmod(s, 360.0f);
  if (s < 0) s += 360.0f;
  if (s >= 360.0f) s = 0.0f;  // canonicalize full circle
  return s;
}

float computeSkyDEC(float mount_angle, unsigned long t_ms) {
  if (!isCalibrated) return mount_angle;
  float delta_m = mount_angle - cal_mount_dec;
  // DEC has negligible sidereal drift for fixed mount
  return cal_sky_dec + delta_m;
}

// ============================================================
// GLOBAL STATE
// ============================================================

Axis axisRA(getGearRatio(MICROSTEPS),  PIN_STEP_RA,  PIN_DIR_RA,  PIN_ENABLE);
Axis axisDEC(getGearRatio(MICROSTEPS), PIN_STEP_DEC, PIN_DIR_DEC, PIN_ENABLE);

int currentMicrosteps = MICROSTEPS;

// No-op for now: microstepping is fixed by the MS1/MS2 hardware jumpers (no UART wiring
// currently installed - see the MICROSTEPPING note in the file header comment), so there's
// nothing to dynamically switch. Kept as a stub rather than removed so the many call sites
// throughout the alignment state machine don't all need to change; re-enable real UART
// switching here (rescale gear ratio + driver.microsteps() call) once UART is wired.
void switchMicrosteps(int newMicrosteps) {
  (void)newMicrosteps;
}

enum TrackingMode { MODE_SIDEREAL, MODE_SOLAR, MODE_LUNAR };
TrackingMode currentMode = MODE_SIDEREAL;

enum AlignState { ALIGN_IDLE, ALIGN_RESETTING_DEC, ALIGN_RA, ALIGN_DEC, ALIGN_WAIT_RA, ALIGN_WAIT_DEC, ALIGN_TRACKING };
AlignState alignState = ALIGN_IDLE;

bool trackingActive = false;
bool timeIsValid = false;

// Manual GoTo sequenced slew state (DEC first then RA, instead of direct both axes)
bool isManualSlewing = false;
float slewTargetRA = 0.0;
float slewTargetDEC = 0.0;

STTime currentUtcTime;
unsigned long lastMillis = 0;
unsigned long lastErrorReport = 0;
unsigned long alignStartTime = 0;
unsigned long alignWaitStartTime = 0;

float targetRA = 0.0;   // Current target sky angles (updated by mode)
float targetDEC = 0.0;

float syncRA = 180.0;   // Last synced position (calibration reference point from SYNC, used to map physical <-> sky)
float syncDEC = 0.0;
unsigned long syncTimeMs = 0;

// Sidereal target (set from GUI input boxes via SET_TARGET for sidereal tracking of arbitrary star).
// This is separate from sync* so that calibration (from e.g. Vega) stays intact, and we slew using the cal
// exactly like sun/moon modes do. computeTargetFromMode returns this for SIDEREAL when set.
float siderealTargetRA = 180.0;
float siderealTargetDEC = 0.0;
bool siderealTargetSet = false;

float lastErrorRA = 0.0;
float lastErrorDEC = 0.0;

// For reporting
float last_sky_ra = 0.0;
float last_sky_dec = 0.0;

char serialBuffer[128];
uint8_t bufIndex = 0;

// ============================================================
// TIME HANDLING
// ============================================================

void setTimeFromValues(int y, int m, int d, int h, int min, int s) {
  currentUtcTime.year   = y;
  currentUtcTime.month  = m;
  currentUtcTime.day    = d;
  currentUtcTime.hour   = h;
  currentUtcTime.minute = min;
  currentUtcTime.second = s;
  timeIsValid = true;
  lastMillis = millis();
}

void updateTimeFromMillis() {
  if (!timeIsValid) return;
  unsigned long now = millis();
  unsigned long elapsed = now - lastMillis;
  lastMillis = now;

  // Advance time (simple, no DST/leap handling needed for astro approx)
  currentUtcTime.second += elapsed / 1000.0;
  while (currentUtcTime.second >= 60.0) {
    currentUtcTime.second -= 60.0;
    currentUtcTime.minute++;
    if (currentUtcTime.minute >= 60) {
      currentUtcTime.minute = 0;
      currentUtcTime.hour++;
      if (currentUtcTime.hour >= 24) {
        currentUtcTime.hour = 0;
        // Day increment simplified (good enough)
        currentUtcTime.day++;
        // Very rough month rollover omitted for brevity - real use should sync time regularly from PC
      }
    }
  }
}

// Get current time struct (for calculations)
STTime getCurrentTime() {
  STTime t = currentUtcTime;
  // Apply fractional second already in struct
  return t;
}

// ============================================================
// POSITION CALCULATION FROM TIME + MODE
// ============================================================

void computeTargetFromMode(float &raOut, float &decOut) {
  STTime t = getCurrentTime();

  if (currentMode == MODE_SOLAR) {
    STLocation loc = { OBS_LON_DEG, OBS_LAT_DEG, 0, 0, 1010, 283 };
    STPosition pos;
    SolTrack(t, loc, &pos, 1, 1, 0, 0);   // degrees, north=0, no refr eq, no dist
    raOut  = pos.rightAscension;
    decOut = pos.declination;
  } else if (currentMode == MODE_LUNAR) {
    MoonPos mp = computeMoonPosition(t, OBS_LAT_DEG, OBS_LON_DEG);
    raOut  = mp.raDeg;
    decOut = mp.decDeg;
  } else {
    // SIDEREAL: use the user-provided target from input boxes (via SET_TARGET) if set;
    // otherwise fall back to last sync (the cal star). This is the fixed sky position to hold.
    // Alignment and tracking then use the calibration (cal_*) + this target to compute
    // the required physical mount moves -- exactly the same way solar/lunar do.
    if (siderealTargetSet) {
      raOut = siderealTargetRA;
      decOut = siderealTargetDEC;
    } else {
      raOut = syncRA;
      decOut = syncDEC;
    }
    // Normalize RA
    while (raOut >= 360.0f) raOut -= 360.0f;
    while (raOut < 0.0f) raOut += 360.0f;
  }
  // Ground safeguard removed per request: any target (even below horizon) is accepted as-is.
  // Mount will point wherever commanded.
}

// ============================================================
// SERIAL COMMUNICATION
// ============================================================

void sendStatus(const char* action, const char* axis, float error = -1.0) {
  Serial.print("STATUS:");
  Serial.print(action);
  Serial.print(",AXIS:");
  Serial.print(axis);
  if (error >= 0) {
    Serial.print(",ERROR:");
    Serial.print(error, 6);
  }
  Serial.println();
}

// Positional CSV, field order fixed as:
//   POS,skyRA,skyDEC,targetRA,targetDEC,mountRA,mountDEC,mode,tracking
// mode is S=SIDEREAL, O=SOLAR, L=LUNAR. tracking is 0/1. Keep tracker_gui.py's POS parser in
// sync with this exact field order if it ever changes.
// ERR_RA/ERR_DEC are not sent - trivially reconstructed on the GUI side as abs(sky - target).
//
// Even at ~55-60 bytes (comfortably under the Mega's 64-byte Serial TX buffer, so Serial.print()
// itself no longer blocks waiting for buffer room - see the 1.8.7 changelog entry), sending the
// whole message in one sendPositionUpdate() call still cost a real, measurable stall: converting
// a float to ASCII (what Serial.print(float, 4) does internally) is genuinely CPU-bound work on
// an 8-bit AVR with no hardware FPU - each call costs roughly 100-300us regardless of message
// size or wire format, and six of them back to back adds up to something comparable to (or
// bigger than) the ~865us step interval AccelStepper needs during a MAX_SLEW_SPEED_DEG_S=2.0
// slew at 8 microsteps. AccelStepper is polled from loop() (see axisRA.update()/axisDEC.update()
// below), not interrupt-driven, so any single call that blocks the CPU for that long delays step
// generation and produces an audible twitch synced exactly to the broadcast - confirmed by user
// testing (silky smooth between broadcasts, twitch exactly on each one, worse at higher rates).
//
// Fix: spread the six float conversions + mode/tracking chars across MULTIPLE loop() iterations
// instead of doing them all in one call - a tiny state machine that emits one or two fields per
// call, advancing to IDLE only once the whole message is out. Since axisRA.update()/
// axisDEC.update() already run unconditionally every loop() iteration (step 5, before this),
// spreading the work this way means the worst-case single stall this adds to any one loop()
// iteration is roughly one float conversion (~100-300us) instead of six (~600-1800us) - a
// real reduction in peak jitter, not just a smaller message. The full message still completes
// well within a fraction of a millisecond of wall-clock time (loop() iterates far faster than
// this normally), just no longer as one uninterrupted block.
//
// This narrows the problem but can't fully eliminate it - AccelStepper's software-timed polling
// approach means ANY sufficiently long single operation in loop() (this, a Serial command
// parse, etc.) can still delay a step. The only way to make stepping fully immune to loop()
// timing is to generate step pulses from a hardware timer interrupt instead of polling - a
// bigger architecture change (see TODO.md) that would remove this entire class of issue for
// good, at any message size or update rate.
enum PosSendState { POS_SEND_IDLE, POS_SEND_RADEC, POS_SEND_TARGET, POS_SEND_MOUNT, POS_SEND_MODE };
PosSendState posSendState = POS_SEND_IDLE;
float posSend_skyRA, posSend_skyDEC, posSend_targetRA, posSend_targetDEC, posSend_mountRA, posSend_mountDEC;

// Called every loop() iteration (see call site) - only actually does anything once per
// positionUpdateIntervalMs (starting a new send) or when a send is already in progress
// (advancing it by one step). Non-blocking either way.
void updatePositionBroadcast() {
  static unsigned long lastPos = 0;
  unsigned long now = millis();

  if (posSendState == POS_SEND_IDLE) {
    if (now - lastPos < positionUpdateIntervalMs) return;
    lastPos = now;
    // Snapshot everything once, at the start of the send sequence, so the message is internally
    // consistent even though it's transmitted across several loop() iterations.
    posSend_mountRA = axisRA.getCurrentAngle();
    posSend_mountDEC = axisDEC.getCurrentAngle();
    posSend_skyRA = isCalibrated ? computeSkyRA(posSend_mountRA, now) : posSend_mountRA;
    posSend_skyDEC = isCalibrated ? computeSkyDEC(posSend_mountDEC, now) : posSend_mountDEC;
    posSend_targetRA = targetRA;
    posSend_targetDEC = targetDEC;
    Serial.print("POS,");
    Serial.print(posSend_skyRA, 4);
    posSendState = POS_SEND_RADEC;
    return;
  }

  switch (posSendState) {
    case POS_SEND_RADEC:
      Serial.print(',');
      Serial.print(posSend_skyDEC, 4);
      posSendState = POS_SEND_TARGET;
      break;
    case POS_SEND_TARGET:
      Serial.print(',');
      Serial.print(posSend_targetRA, 4);
      Serial.print(',');
      Serial.print(posSend_targetDEC, 4);
      posSendState = POS_SEND_MOUNT;
      break;
    case POS_SEND_MOUNT:
      Serial.print(',');
      Serial.print(posSend_mountRA, 4);
      Serial.print(',');
      Serial.print(posSend_mountDEC, 4);
      posSendState = POS_SEND_MODE;
      break;
    case POS_SEND_MODE:
      Serial.print(',');
      switch (currentMode) {
        case MODE_SIDEREAL: Serial.print('S'); break;
        case MODE_SOLAR:    Serial.print('O'); break;
        case MODE_LUNAR:    Serial.print('L'); break;
      }
      Serial.print(',');
      Serial.println(trackingActive ? '1' : '0');
      posSendState = POS_SEND_IDLE;
      break;
    default:
      posSendState = POS_SEND_IDLE;
      break;
  }
}

// Synchronous, immediate, full POS send - same field layout/order as updatePositionBroadcast()
// but not spread across loop() iterations. Only for rare one-off events (boot, SET_TIME,
// SET_LOCATION, offset-applied, etc.) that need the GUI to see fresh state right away and aren't
// happening during a time-critical slew, so the ~1ms this blocks for doesn't matter here the way
// it did for the periodic broadcast (see the 1.8.9 changelog entry / comment above
// updatePositionBroadcast()). Also resets any in-progress incremental send back to idle, since
// this supersedes it with a complete, up-to-date message anyway.
void sendPositionUpdateNow() {
  posSendState = POS_SEND_IDLE;
  unsigned long now = millis();
  float mountRA = axisRA.getCurrentAngle();
  float mountDEC = axisDEC.getCurrentAngle();
  float skyRA = isCalibrated ? computeSkyRA(mountRA, now) : mountRA;
  float skyDEC = isCalibrated ? computeSkyDEC(mountDEC, now) : mountDEC;

  Serial.print("POS,");
  Serial.print(skyRA, 4);
  Serial.print(',');
  Serial.print(skyDEC, 4);
  Serial.print(',');
  Serial.print(targetRA, 4);
  Serial.print(',');
  Serial.print(targetDEC, 4);
  Serial.print(',');
  Serial.print(mountRA, 4);
  Serial.print(',');
  Serial.print(mountDEC, 4);
  Serial.print(',');
  switch (currentMode) {
    case MODE_SIDEREAL: Serial.print('S'); break;
    case MODE_SOLAR:    Serial.print('O'); break;
    case MODE_LUNAR:    Serial.print('L'); break;
  }
  Serial.print(',');
  Serial.println(trackingActive ? '1' : '0');
}

float estimateMovementTime(float deltaDeg, float speedDegS, float accelDegS2 = 1.0f) {  // default matches current 5s ramp (1 °/s²)
  float absD = fabs(deltaDeg);
  if (speedDegS < 0.001f) speedDegS = 0.5f;
  if (accelDegS2 < 0.01f) accelDegS2 = 1.0f;
  float tRamp = speedDegS / accelDegS2;
  float dRamp = 0.5f * speedDegS * tRamp; // distance for one ramp (accel or decel)
  float t;
  if (absD <= (2.0f * dRamp)) {
    // Triangular profile (never reaches full speed)
    t = sqrt(2.0 * absD / accelDegS2);
  } else {
    float tCruise = (absD - 2.0f * dRamp) / speedDegS;
    t = tCruise + 2.0f * tRamp;
  }
  return t + 0.5f; // extra margin for settling, serial, etc.
}

// ============================================================
// VERBOSE DEBUG DUMP
// Call this when debugVerbose is on, or on demand.
// Prints a ton of internal variables over serial to help diagnose
// sync / calibration / drift / position reporting issues.
// GUI already logs any line starting with "DEBUG:"
// ============================================================
void dumpDebugState(const char* trigger) {
  unsigned long now = millis();
  float mRA = axisRA.getCurrentAngle();
  float mDEC = axisDEC.getCurrentAngle();
  float sRA = isCalibrated ? computeSkyRA(mRA, now) : mRA;
  float sDEC = isCalibrated ? computeSkyDEC(mDEC, now) : mDEC;
  float dt = isCalibrated ? (now - cal_time_ms) / 1000.0f : 0.0f;
  float drift = isCalibrated ? (SIDEREAL_RATE_DEG_S * dt) : 0.0f;

  Serial.print("DEBUG:STATE,trigger:");
  Serial.print(trigger);
  Serial.print(",firmware:");
  Serial.print(FIRMWARE_VERSION);
  Serial.print(",millis:");
  Serial.print(now);
  Serial.print(",isCalibrated:");
  Serial.print(isCalibrated ? 1 : 0);
  Serial.print(",cal_mount_ra:");
  Serial.print(cal_mount_ra, 6);
  Serial.print(",cal_sky_ra:");
  Serial.print(cal_sky_ra, 6);
  Serial.print(",cal_mount_dec:");
  Serial.print(cal_mount_dec, 6);
  Serial.print(",cal_sky_dec:");
  Serial.print(cal_sky_dec, 6);
  Serial.print(",cal_time_ms:");
  Serial.print(cal_time_ms);
  Serial.print(",dt_sec:");
  Serial.print(dt, 6);
  Serial.print(",drift_deg:");
  Serial.print(drift, 6);
  Serial.print(",mountRA:");
  Serial.print(mRA, 6);
  Serial.print(",mountDEC:");
  Serial.print(mDEC, 6);
  Serial.print(",skyRA:");
  Serial.print(sRA, 6);
  Serial.print(",skyDEC:");
  Serial.print(sDEC, 6);
  Serial.print(",targetRA:");
  Serial.print(targetRA, 6);
  Serial.print(",targetDEC:");
  Serial.print(targetDEC, 6);
  Serial.print(",syncRA:");
  Serial.print(syncRA, 6);
  Serial.print(",syncDEC:");
  Serial.print(syncDEC, 6);
  Serial.print(",syncTimeMs:");
  Serial.print(syncTimeMs);
  Serial.print(",siderealTargetRA:");
  Serial.print(siderealTargetRA, 6);
  Serial.print(",siderealTargetDEC:");
  Serial.print(siderealTargetDEC, 6);
  Serial.print(",siderealTargetSet:");
  Serial.print(siderealTargetSet ? 1 : 0);
  Serial.print(",trackingActive:");
  Serial.print(trackingActive ? 1 : 0);
  Serial.print(",alignState:");
  Serial.print((int)alignState);
  Serial.print(",mode:");
  switch (currentMode) {
    case MODE_SIDEREAL: Serial.print("SIDEREAL"); break;
    case MODE_SOLAR:    Serial.print("SOLAR"); break;
    case MODE_LUNAR:    Serial.print("LUNAR"); break;
  }
  Serial.print(",posIntervalMs:");
  Serial.print(positionUpdateIntervalMs);
  Serial.print(",timeValid:");
  Serial.print(timeIsValid ? 1 : 0);
  Serial.print(",lat:");
  Serial.print(OBS_LAT_DEG, 6);
  Serial.print(",lon:");
  Serial.print(OBS_LON_DEG, 6);
  Serial.print(",currentMicrosteps:");
  Serial.print(currentMicrosteps);
  Serial.print(",isManualSlewing:");
  Serial.print(isManualSlewing ? 1 : 0);
  Serial.println();  // end of big debug line
}

void parseAndExecuteCommand(char* cmd) {
  // Expected formats:
  // CMD,MODE,SIDEREAL
  // CMD,MODE,SOLAR
  // CMD,MODE,LUNAR
  // CMD,START_TRACKING
  // CMD,STOP
  // CMD,SYNC,RA:123.45,DEC:-12.3   (calibration: sets both cal ref + initial sidereal target)
  // CMD,SYNC_OFFSET,RA:0.1,DEC:-0.05   (nudge the mount by this amount to fine-tune pointing;
  //                                     target/sync values are untouched, so this only corrects
  //                                     where the CURRENT target sits, not future sun/moon/star targets)
  // CMD,SET_TARGET,RA:xx,DEC:yy   (for SIDEREAL: sets the fixed star to track from input boxes; does not affect cal)
  // CMD,GOTO,RA:45.0,DEC:30.0
  // CMD,SET_TIME,Y:2026,M:7,D:11,h:20,min:30,s:15
  // CMD,SET_LOCATION,LAT:xx.x,LON:yy.y   (real GPS coords from GUI)
  // CMD,SET_POS_UPDATE,MS:200   // change how often Arduino broadcasts POS (10-5000)
  // CMD,DEBUG,ON   CMD,DEBUG,OFF   CMD,DEBUG,DUMP   // heavy internal variable dump for debugging (sync, cal, drift etc)
  // CMD,SAVE_LOCATION_EEPROM   // save current location to EEPROM
  // CMD,LOAD_LOCATION_EEPROM   // load location from EEPROM
  // CMD,QUERY_LOCATION         // query current location

  char line[128];
  strncpy(line, cmd, 127);
  line[127] = '\0';

  // Robust command detection: locate "CMD," anywhere. This prevents missed commands due to
  // stale/partial serial data left in RX buffer (a regression risk introduced when we switched
  // to dtr=false/rts=false to make Arduino keep its state independent of GUI reconnects).
  char* p = strstr(line, "CMD,");
  if (!p) return;

  // Extract action non-destructively from the actual command start.
  char action[32];
  if (sscanf(p, "CMD,%[^,]", action) != 1) return;

  // Debug: confirm command reached the parser (helps diagnose stale buffer / independence issues)
  Serial.print("DEBUG:CMD_ACTION:");
  Serial.println(action);

  if (strcmp(action, "MODE") == 0) {
    char mode[20];
    if (sscanf(p, "CMD,MODE,%s", mode) == 1) {
      if (strcmp(mode, "SIDEREAL") == 0) currentMode = MODE_SIDEREAL;
      else if (strcmp(mode, "SOLAR") == 0) currentMode = MODE_SOLAR;
      else if (strcmp(mode, "LUNAR") == 0) currentMode = MODE_LUNAR;
      // Recompute the target for the new mode immediately, not just when already tracking.
      // sendPositionUpdate() reports targetRA/DEC unconditionally (tracking or not), so
      // leaving it stale here meant clients saw the OLD mode's target in POS updates until
      // tracking actually (re)started - e.g. a GUI client's "is the new target close to the
      // current position?" check comparing against that stale value right after a mode
      // switch, wrongly concluding a brand new Sun/Moon target was "close" because it was
      // still comparing against the previous mode's target.
      computeTargetFromMode(targetRA, targetDEC);
    }
  }
  else if (strcmp(action, "START_TRACKING") == 0) {
    // "action" is truncated at the first comma by the sscanf above, so any
    // suffix like ",SKIP_DEC_RESET" only exists in the full line (p) -- check there.
    bool skipDecReset = strstr(p, "SKIP_DEC_RESET") != NULL;
    startAlignmentSequence(skipDecReset);
  }
  else if (strcmp(action, "STOP") == 0) {
    stopAll();
  }
  else if (strcmp(action, "STOP_SLEW") == 0) {
    isManualSlewing = false;
    float curRA = axisRA.getCurrentAngle();
    float curDEC = axisDEC.getCurrentAngle();
    axisRA.setTargetPosition(curRA, false);
    axisDEC.setTargetPosition(curDEC, false);
    switchMicrosteps(TRACKING_MICROSTEPS);
    sendStatus("SLEW_STOPPED", "BOTH");
  }
  else if (strcmp(action, "SYNC") == 0) {
    float ra = 0, dec = 0;
    // Robust extraction (strstr + atof + sscanf fallback for zero). Tolerates prefix garbage
    // from serial buffers (no-reset reconnects) and is consistent with the GOTO handler.
    const char* ra_p = strstr(line, "RA:");
    const char* dec_p = strstr(line, "DEC:");
    if (ra_p) ra = atof(ra_p + 3);
    if (dec_p) dec = atof(dec_p + 4);
    if (ra == 0.0f && ra_p) { float tmp; if (sscanf(ra_p, "RA:%f", &tmp)==1) ra=tmp; }
    if (dec == 0.0f && dec_p) { float tmp; if (sscanf(dec_p, "DEC:%f", &tmp)==1) dec=tmp; }

    // Record calibration: current physical mount angle corresponds to this sky position at this time.
    // NO rebasing of steps. Sky is computed from mount + time drift.
    // This allows sky position to drift with Earth rotation when mount is fixed (not tracking).
    cal_mount_ra = axisRA.getCurrentAngle();
    cal_sky_ra = ra;
    cal_mount_dec = axisDEC.getCurrentAngle();
    cal_sky_dec = dec;
    cal_time_ms = millis();
    isCalibrated = true;

    // Debug info so we can see in GUI log whether Arduino actually received + accepted the sync values
    Serial.print("DEBUG:SYNC_PARSED ra=");
    Serial.print(ra, 6);
    Serial.print(" dec=");
    Serial.print(dec, 6);
    Serial.print(" cal_mount_ra=");
    Serial.print(cal_mount_ra, 6);
    Serial.print(" isCalibrated=");
    Serial.println(isCalibrated ? "1" : "0");

    // Full verbose state dump (will only be very noisy if debugVerbose was turned on)
    if (debugVerbose) {
      dumpDebugState("AFTER_SYNC");
    } else {
      // Always emit a compact one for sync troubleshooting even if not in full verbose
      Serial.print("DEBUG:SYNC_CAL state,cal_mount_ra=");
      Serial.print(cal_mount_ra, 6);
      Serial.print(",cal_sky_ra=");
      Serial.print(cal_sky_ra, 6);
      Serial.print(",cal_mount_dec=");
      Serial.print(cal_mount_dec, 6);
      Serial.print(",cal_sky_dec=");
      Serial.print(cal_sky_dec, 6);
      Serial.print(",isCal=1,millis=");
      Serial.println(millis());
    }

    // Update high-level targets for display / tracking start
    targetRA = ra;
    targetDEC = dec;
    syncRA = ra;
    syncDEC = dec;
    syncTimeMs = millis();

    // Also seed the sidereal target so "start tracking" right after sync follows the synced star
    // (unless user sends SET_TARGET for a different star before START_TRACKING).
    siderealTargetRA = ra;
    siderealTargetDEC = dec;
    siderealTargetSet = true;

    // Stop any movement
    axisRA.setTargetPosition(cal_mount_ra, false);
    axisDEC.setTargetPosition(cal_mount_dec, false);

    trackingActive = false;
    alignState = ALIGN_IDLE;
    isManualSlewing = false;

    // For the immediate sync confirmation report, refresh cal_time so dt~0 and sky snaps to exactly
    // the provided star coordinates. Ongoing periodic reports will use the original sync instant
    // for correct future drift.
    unsigned long saved_cal_time = cal_time_ms;
    cal_time_ms = millis();

    // Extra debug: what sky value is the immediate POS going to contain?
    float dbg_snap_ra = computeSkyRA(axisRA.getCurrentAngle(), millis());
    float dbg_snap_dec = computeSkyDEC(axisDEC.getCurrentAngle(), millis());
    Serial.print("DEBUG:SYNC_SNAP_SENDING RA=");
    Serial.print(dbg_snap_ra, 6);
    Serial.print(" DEC=");
    Serial.println(dbg_snap_dec, 6);

    sendPositionUpdateNow();
    cal_time_ms = saved_cal_time;

    if (debugVerbose) {
      dumpDebugState("SYNC_SNAP");
    }

    Serial.println("STATUS:SYNCED");
    Serial.flush();  // ensure the response (POS + STATUS) is pushed out right away
  }
  else if (strcmp(action, "SYNC_OFFSET") == 0) {
    // Apply offset to current calibration for fine-tuning alignment
    float ra_offset = 0, dec_offset = 0;
    const char* ra_p = strstr(line, "RA:");
    const char* dec_p = strstr(line, "DEC:");
    if (ra_p) ra_offset = atof(ra_p + 3);
    if (dec_p) dec_offset = atof(dec_p + 4);
    if (ra_offset == 0.0f && ra_p) { float tmp; if (sscanf(ra_p, "RA:%f", &tmp)==1) ra_offset=tmp; }
    if (dec_offset == 0.0f && dec_p) { float tmp; if (sscanf(dec_p, "DEC:%f", &tmp)==1) dec_offset=tmp; }

    if (isCalibrated) {
      // Nudge the mount-side calibration reference only. Mount position (for ANY mode) is
      // always computed as: cal_mount + (target - cal_sky) + drift. Previously this offset
      // was added to BOTH cal_sky_ra/dec AND to targetRA/DEC (+ siderealTarget/sync) by the
      // same amount, which cancels out in that formula -- the mount never physically moved,
      // while cal_sky_ra/dec (the single shared mount<->sky reference used by EVERY mode:
      // sidereal, solar, lunar) permanently drifted by the offset. That silently mis-pointed
      // every subsequent target (sun/moon/other stars), not just the one being tracked.
      // Shifting cal_mount_ra/dec instead actually slews the mount by the requested amount,
      // and the live error settles back to ~0 at the (unchanged) target once it arrives --
      // exactly what a "nudge to fine-tune alignment" should do, with no side effect on
      // future targets.
      cal_mount_ra += ra_offset;
      cal_mount_dec += dec_offset;

      Serial.print("DEBUG:SYNC_OFFSET_APPLIED ra_offset=");
      Serial.print(ra_offset, 6);
      Serial.print(" dec_offset=");
      Serial.print(dec_offset, 6);
      Serial.print(" new_cal_mount_ra=");
      Serial.print(cal_mount_ra, 6);
      Serial.print(" new_cal_mount_dec=");
      Serial.println(cal_mount_dec, 6);

      if (debugVerbose) {
        dumpDebugState("AFTER_OFFSET");
      }

      sendPositionUpdateNow();
      Serial.println("STATUS:OFFSET_APPLIED");
      Serial.flush();
    } else {
      Serial.println("ERROR:NOT_CALIBRATED");
    }
  }
  else if (strcmp(action, "SET_TARGET") == 0) {
    // For sidereal tracking: set the desired fixed sky position for the star to track.
    // IMPORTANT: does NOT touch syncRA/syncDEC or cal_*  -- sync/cal is ONLY from SYNC command
    // (the reference calibration, e.g. on Vega). This lets alignment use exactly the same
    // mount target computation as sun/moon: desired_m = cal_mount + (target - cal_sky) + drift
    float ra = 0, dec = 0;
    const char* ra_p = strstr(line, "RA:");
    const char* dec_p = strstr(line, "DEC:");
    if (ra_p) ra = atof(ra_p + 3);
    if (dec_p) dec = atof(dec_p + 4);
    if (ra == 0.0f && ra_p) { float tmp; if (sscanf(ra_p, "RA:%f", &tmp)==1) ra=tmp; }
    if (dec == 0.0f && dec_p) { float tmp; if (sscanf(dec_p, "DEC:%f", &tmp)==1) dec=tmp; }

    siderealTargetRA = ra;
    siderealTargetDEC = dec;
    siderealTargetSet = true;
    targetRA = ra;
    targetDEC = dec;
  }
  else if (strcmp(action, "GOTO") == 0) {
    float ra = 0, dec = 0;
    // Robust extraction (same as SYNC).
    const char* ra_p = strstr(line, "RA:");
    const char* dec_p = strstr(line, "DEC:");
    if (ra_p) ra = atof(ra_p + 3);
    if (dec_p) dec = atof(dec_p + 4);
    if (ra == 0.0f && ra_p) { float tmp; if (sscanf(ra_p, "RA:%f", &tmp)==1) ra=tmp; }
    if (dec == 0.0f && dec_p) { float tmp; if (sscanf(dec_p, "DEC:%f", &tmp)==1) dec=tmp; }

    // Ground safeguard removed: use the requested RA/DEC exactly (even below horizon).
    stopAlignment();
    trackingActive = false;
    isManualSlewing = false;

    axisRA.enable(true);
    axisDEC.enable(true);

    switchMicrosteps(SLEW_MICROSTEPS);

    // ra, dec are desired sky targets. Compute required mount physical targets.
    // Note: ra/dec have already been clamped to horizon by the safety code above.
    if (isCalibrated) {
      // Lead time compensation so we arrive at the moving (sidereal) target.
      // LEAD_COEFFICIENT is a margin in SECONDS (see its doc comment) - it must be added
      // directly to the time total, not scaled by SIDEREAL_RATE_DEG_S first (that would turn
      // a "2 extra seconds" margin into "2 extra (mislabeled) degrees").
      float curSkyRA = computeSkyRA(axisRA.getCurrentAngle(), millis());
      // Direct (unwrapped) delta - see the comment in runAlignmentSequence's ALIGN_RESETTING_DEC
      // case: the RA cable can't tolerate crossing the RA=0/360 seam at all, so this must NOT
      // be shortest-pathed even if that means a longer rotation.
      float deltaRA = ra - curSkyRA;
      float moveTime = estimateMovementTime(deltaRA, MAX_SLEW_SPEED_DEG_S, SLEW_ACCEL_DEG_S2);
      float lead = moveTime + LEAD_COEFFICIENT;
      float desired_m_ra = axisRA.getCurrentAngle() + deltaRA + SIDEREAL_RATE_DEG_S * lead;
      float desired_m_dec = cal_mount_dec + (dec - cal_sky_dec);
      // Sequenced: DEC first (physical mount angle). Ensure high slew speed is commanded.
      axisDEC.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
      axisDEC.setAcceleration(SLEW_ACCEL_DEG_S2);
      axisRA.setTargetPosition(axisRA.getCurrentAngle(), false);
      axisDEC.setTargetPosition(desired_m_dec, true);
      slewTargetRA = desired_m_ra;
      slewTargetDEC = desired_m_dec;
      isManualSlewing = true;
      sendStatus("SLEWING", "DEC");
    } else {
      // Fallback (no calibration) - still use the (clamped) ra/dec as physical targets
      axisRA.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
      axisRA.setAcceleration(SLEW_ACCEL_DEG_S2);
      axisDEC.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
      axisDEC.setAcceleration(SLEW_ACCEL_DEG_S2);
      axisRA.setTargetPosition(ra, true);
      axisDEC.setTargetPosition(dec, true);
    }

    targetRA = ra;
    targetDEC = dec;

    sendStatus("SLEWING", "DEC");
  }
  else if (strcmp(action, "SET_TIME") == 0) {
    int y=2026, mo=1, d=1, h=0, mi=0; float s=0;
    // Use strstr + sscanf so we don't depend on strtok state or exact full match order.
    const char* p;
    int val; float fval;
    if ((p = strstr(line, "Y:"))) sscanf(p, "Y:%d", &y);
    if ((p = strstr(line, "M:"))) sscanf(p, "M:%d", &mo);
    if ((p = strstr(line, "D:"))) sscanf(p, "D:%d", &d);
    if ((p = strstr(line, "h:"))) sscanf(p, "h:%d", &h);
    if ((p = strstr(line, "min:"))) sscanf(p, "min:%d", &mi);
    if ((p = strstr(line, "s:"))) sscanf(p, "s:%f", &fval);
    setTimeFromValues(y, mo, d, h, mi, (int)s);
    Serial.println("STATUS:TIME_SET");
    sendPositionUpdateNow();  // send current state immediately so GUI can restore on reconnect without reset
  }
  else if (strcmp(action, "SET_LOCATION") == 0) {
    // Robust parse like SYNC/SET_TARGET/GOTO to tolerate any stale buffer junk.
    const char* lat_p = strstr(line, "LAT:");
    const char* lon_p = strstr(line, "LON:");
    if (lat_p) OBS_LAT_DEG = atof(lat_p + 4);
    if (lon_p) OBS_LON_DEG = atof(lon_p + 4);
    // Fallback for zero values (rare for lat/lon)
    if (OBS_LAT_DEG == 0.0f && lat_p) {
      float tmp; if (sscanf(lat_p, "LAT:%f", &tmp) == 1) OBS_LAT_DEG = tmp;
    }
    if (OBS_LON_DEG == 0.0f && lon_p) {
      float tmp; if (sscanf(lon_p, "LON:%f", &tmp) == 1) OBS_LON_DEG = tmp;
    }
    Serial.print("STATUS:LOCATION_SET,LAT:");
    Serial.print(OBS_LAT_DEG, 6);
    Serial.print(",LON:");
    Serial.println(OBS_LON_DEG, 6);
    sendPositionUpdateNow();  // send current state immediately so GUI can restore on reconnect
  }
  else if (strcmp(action, "SET_POS_UPDATE") == 0) {
    const char* p;
    int ms = 0;
    if ((p = strstr(line, "MS:"))) {
      if (sscanf(p, "MS:%d", &ms) == 1) {
        if (ms >= 10 && ms <= 5000) {
          positionUpdateIntervalMs = (unsigned long)ms;
          Serial.print("STATUS:UPDATE_RATE,MS:");
          Serial.println(positionUpdateIntervalMs);
        }
      }
    }
  }
  else if (strcmp(action, "DEBUG") == 0) {
    // CMD,DEBUG,ON   / OFF   / DUMP
    // Enables heavy variable dump for debugging sync/calibration/position issues.
    // Use strstr to be robust against any stale prefix bytes on the line.
    const char* dbg = strstr(line, "DEBUG,");
    const char* sub = NULL;
    if (dbg) {
      sub = strstr(dbg + 6, ",");  // look for comma after "DEBUG,"
      if (sub) sub += 1;
      else sub = dbg + 6;  // no more comma, the rest is the value e.g. "DUMP"
    }
    if (sub) {
      if (strncmp(sub, "ON", 2) == 0 || strncmp(sub, "1", 1) == 0) {
        debugVerbose = true;
        lastDebugDumpMs = 0;
        Serial.println("DEBUG:VERBOSE_ON");
        dumpDebugState("ENABLED");
      } else if (strncmp(sub, "OFF", 3) == 0 || strncmp(sub, "0", 1) == 0) {
        debugVerbose = false;
        Serial.println("DEBUG:VERBOSE_OFF");
      } else if (strncmp(sub, "DUMP", 4) == 0) {
        dumpDebugState("MANUAL_DUMP");
      }
    } else {
      // bare CMD,DEBUG or no sub -> toggle
      debugVerbose = !debugVerbose;
      if (debugVerbose) {
        dumpDebugState("TOGGLED_ON");
      } else {
        Serial.println("DEBUG:VERBOSE_OFF");
      }
    }
  }
  else if (strcmp(action, "SAVE_LOCATION_EEPROM") == 0) {
    saveLocationToEEPROM();
  }
  else if (strcmp(action, "LOAD_LOCATION_EEPROM") == 0) {
    loadLocationFromEEPROM();
  }
  else if (strcmp(action, "QUERY_LOCATION") == 0) {
    queryLocation();
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufIndex > 0) {
        serialBuffer[bufIndex] = '\0';
        parseAndExecuteCommand(serialBuffer);
        bufIndex = 0;
      }
    } else if (bufIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[bufIndex++] = c;
    }
  }
}

// ============================================================
// ALIGNMENT AND TRACKING LOGIC
// ============================================================

void startAlignmentSequence(bool skipDecReset) {
  if (alignState != ALIGN_IDLE && alignState != ALIGN_TRACKING) return;

  trackingActive = false;
  isManualSlewing = false;
  axisRA.enable(true);
  axisDEC.enable(true);

  switchMicrosteps(SLEW_MICROSTEPS);

  // Calculate the actual target (sidereal/moon/sun) *before* commanding the axes.
  computeTargetFromMode(targetRA, targetDEC);

  // Always go through the same sequential state machine (RESETTING_DEC -> RA -> WAIT_RA ->
  // DEC -> WAIT_DEC -> TRACKING), one axis moving at a time - never both at once, so cabling
  // is never put through concurrent RA+DEC motion regardless of which path got us here.
  //
  // skipDecReset only changes what DEC does *during* the "RESETTING_DEC" step: normally it
  // slews all the way out to 0 Sky DEC and back as a safe, known starting point; when the
  // target is already close (skipDecReset), that round trip is pure wasted time, so DEC
  // instead just holds its current position - which is already "not moving" the very next
  // loop() tick, so the state machine falls through into the RA alignment step immediately.
  // (This used to be a separate ALIGN_SLEWING_TO_TARGET state that moved RA and DEC at the
  // same time - that's exactly the "both axes move together" behavior that risks cable
  // tangle, so it's gone; skipping the DEC round-trip no longer means skipping sequencing.)
  alignState = ALIGN_RESETTING_DEC;
  sendStatus("RESETTING", "DEC");

  if (skipDecReset) {
    axisDEC.setTargetPosition(axisDEC.getCurrentAngle(), false);
  } else if (isCalibrated) {
    float targetSkyDec = 0.0f;
    float desiredMDec = cal_mount_dec + (targetSkyDec - cal_sky_dec);
    axisDEC.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
    axisDEC.setAcceleration(SLEW_ACCEL_DEG_S2);
    axisDEC.setTargetPosition(desiredMDec, true);
  } else {
    // No cal yet: just go to 0 as physical angle (fallback)
    axisDEC.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
    axisDEC.setAcceleration(SLEW_ACCEL_DEG_S2);
    axisDEC.setTargetPosition(0.0f, true);
  }
  // Keep RA where it is during the DEC step (whichever branch above ran).
  axisRA.setTargetPosition(axisRA.getCurrentAngle(), false);

  alignStartTime = millis();
}

void stopAlignment() {
  alignState = ALIGN_IDLE;
  trackingActive = false;
  // Leave motors enabled or not? Keep enabled.
}

void stopAll() {
  trackingActive = false;
  alignState = ALIGN_IDLE;
  isManualSlewing = false;
  // Stop by setting current as target
  float curRA = axisRA.getCurrentAngle();
  float curDEC = axisDEC.getCurrentAngle();
  axisRA.setTargetPosition(curRA, false);
  axisDEC.setTargetPosition(curDEC, false);
  switchMicrosteps(TRACKING_MICROSTEPS);
  sendStatus("STOPPED", "BOTH");
}

void runAlignmentSequence() {
  if (alignState == ALIGN_IDLE) return;

  unsigned long now = millis();

  switch (alignState) {
    case ALIGN_RESETTING_DEC:
      if (!axisDEC.isMoving() && !axisRA.isMoving()) {
        // DEC reset to 0 Sky DEC complete. Now align RA ONLY (DEC is deliberately left
        // parked at its reset position - it gets its real move later, in ALIGN_WAIT_RA's
        // transition to ALIGN_DEC, once RA has arrived. Commanding both here used to make
        // both axes slew at once instead of sequentially).
        alignState = ALIGN_RA;
        sendStatus("ALIGNING", "RA");

        // Compute future target for RA (for SIDEREAL this will be the user target from boxes or synced star)
        float futureRA, futureDEC;
        computeTargetFromMode(futureRA, futureDEC);

        // Direct (unwrapped) sky-RA distance from where RA currently is to the target. Both
        // cur_sky_ra and futureRA are already normalized to [0,360), so this plain subtraction
        // is the delta that moves straight between them WITHOUT ever crossing the RA=0/360
        // seam - it can be a large angle (up to just under 360 deg) when the two points are far
        // apart "the short way", but it never wraps. The RA cable can't tolerate crossing that
        // seam at all, in either direction, regardless of the resulting distance being longer.
        float cur_sky_ra = computeSkyRA(axisRA.getCurrentAngle(), now);
        float deltaRA = futureRA - cur_sky_ra;
        float moveTime = estimateMovementTime(deltaRA, MAX_SLEW_SPEED_DEG_S, SLEW_ACCEL_DEG_S2);

        float desired_mount_ra;
        if (currentMode == MODE_SIDEREAL) {
          // Sidereal: move by the direct (non-wrapping) distance to the (fixed) target, plus a small
          // forward margin so the axis is already slightly ahead of sidereal drift once it
          // arrives, instead of needing an immediate correction the moment WAIT_RA starts.
          // NOTE: previously this added `moveTime` (a value in SECONDS) directly onto a sky
          // position in DEGREES, and never referenced the target's distance at all - harmless
          // for the sub-second corrections seen during normal continuous tracking, but for any
          // real slew to a new, far-away target (e.g. re-targeting from Vega to Tau Ceti) it
          // produced a nonsensical multi-ten-degree offset that had nothing to do with the
          // actual target, which is exactly the "RA goes to the wrong place" bug.
          float leadMarginDeg = SIDEREAL_RATE_DEG_S * (moveTime + LEAD_COEFFICIENT);
          desired_mount_ra = axisRA.getCurrentAngle() + deltaRA + leadMarginDeg;
        } else {
          // Solar/Lunar: no lead needed (target moves, but we re-compute continuously)
          desired_mount_ra = axisRA.getCurrentAngle() + deltaRA;
        }
        axisRA.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
        axisRA.setAcceleration(SLEW_ACCEL_DEG_S2);
        axisRA.setTargetPosition(desired_mount_ra, true);
      }
      break;

    case ALIGN_RA:
      if (!axisRA.isMoving()) {
        float cur_sky = computeSkyRA(axisRA.getCurrentAngle(), now);
        lastErrorRA = abs(cur_sky - targetRA);
        sendStatus("WAITING", "RA", lastErrorRA);
        alignState = ALIGN_WAIT_RA;
        lastErrorReport = now;
        alignWaitStartTime = now;
        // Switch to fine tracking steps for the wait/monitor phase.
        // Sidereal will follow position target (updated in the wait loop) for accurate rate.
        switchMicrosteps(TRACKING_MICROSTEPS);
        if (currentMode == MODE_SIDEREAL && isCalibrated) {
          float elapsed = (millis() - cal_time_ms) / 1000.0;
          float des = cal_mount_ra + (targetRA - cal_sky_ra) + SIDEREAL_RATE_DEG_S * elapsed;
          axisRA.setTargetPosition(des, false);
        }
      }
      break;

    case ALIGN_WAIT_RA:
      // Keep reporting error at 10Hz
      if (now - lastErrorReport >= ERROR_UPDATE_MS) {
        float cur_sky = computeSkyRA(axisRA.getCurrentAngle(), now);
        lastErrorRA = abs(cur_sky - targetRA);
        sendStatus("WAITING", "RA", lastErrorRA);
        lastErrorReport = now;
        // During wait for sidereal, keep following the time-dependent mount position (like tracking).
        // This ensures we don't rely only on velocity mode and error doesn't grow while waiting.
        if (currentMode == MODE_SIDEREAL && isCalibrated) {
          float elapsed = (millis() - cal_time_ms) / 1000.0;
          float des = cal_mount_ra + (targetRA - cal_sky_ra) + SIDEREAL_RATE_DEG_S * elapsed;
          axisRA.setTargetPosition(des, false);
        }
      }
      if (lastErrorRA <= ACCEPTABLE_ERROR_DEG || (now - alignWaitStartTime > 3000)) {
        // Proceed even if slightly over threshold after timeout (prediction may have small error; tracking will hold)
        sendStatus("TRACKING_STARTED", "RA");
        // Now do DEC - switch back to slew microsteps for fast/accurate DEC movement if needed.
        switchMicrosteps(SLEW_MICROSTEPS);
        alignState = ALIGN_DEC;
        sendStatus("ALIGNING", "DEC");

        float futureRA2, futureDEC2;
        computeTargetFromMode(futureRA2, futureDEC2);
        float cur_sky_dec = computeSkyDEC(axisDEC.getCurrentAngle(), now);
        float deltaDEC = futureDEC2 - cur_sky_dec;
        float moveTime = estimateMovementTime(deltaDEC, MAX_SLEW_SPEED_DEG_S, SLEW_ACCEL_DEG_S2);
        float lead = moveTime + (0.0001f * LEAD_COEFFICIENT); // dec motion very slow

        // Ground safeguard removed: direct target for DEC move.
        float predDEC = futureDEC2;

        targetDEC = predDEC;
        axisDEC.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
        axisDEC.setAcceleration(SLEW_ACCEL_DEG_S2);
        if (isCalibrated) {
          float desired_m = cal_mount_dec + (futureDEC2 - cal_sky_dec);
          axisDEC.setTargetPosition(desired_m, true);
        } else {
          axisDEC.setTargetPosition(futureDEC2, true);
        }
      }
      break;

    case ALIGN_DEC:
      if (!axisDEC.isMoving()) {
        float cur_sky = computeSkyDEC(axisDEC.getCurrentAngle(), now);
        lastErrorDEC = abs(cur_sky - targetDEC);
        sendStatus("WAITING", "DEC", lastErrorDEC);
        alignState = ALIGN_WAIT_DEC;
        lastErrorReport = now;
        alignWaitStartTime = now;
        // Fine steps for precise holding in wait phase.
        switchMicrosteps(TRACKING_MICROSTEPS);
        if (currentMode == MODE_SIDEREAL && isCalibrated) {
          float elapsed = (millis() - cal_time_ms) / 1000.0;
          float des = cal_mount_ra + (targetRA - cal_sky_ra) + SIDEREAL_RATE_DEG_S * elapsed;
          axisRA.setTargetPosition(des, false);
        }
      }
      break;

    case ALIGN_WAIT_DEC:
      if (now - lastErrorReport >= ERROR_UPDATE_MS) {
        float cur_sky = computeSkyDEC(axisDEC.getCurrentAngle(), now);
        lastErrorDEC = abs(cur_sky - targetDEC);
        sendStatus("WAITING", "DEC", lastErrorDEC);
        lastErrorReport = now;
        // Keep RA following sidereal rate while waiting on DEC.
        if (currentMode == MODE_SIDEREAL && isCalibrated) {
          float elapsed = (millis() - cal_time_ms) / 1000.0;
          float des_ra = cal_mount_ra + (targetRA - cal_sky_ra) + SIDEREAL_RATE_DEG_S * elapsed;
          axisRA.setTargetPosition(des_ra, false);
        }
      }
      if (lastErrorDEC <= ACCEPTABLE_ERROR_DEG || (now - alignWaitStartTime > 3000)) {
        // Proceed even if slightly over threshold after timeout
        sendStatus("TRACKING_STARTED", "DEC");
        alignState = ALIGN_TRACKING;
        trackingActive = true;
        switchMicrosteps(TRACKING_MICROSTEPS);
        // Prime targets - compute logical target (exact for sidereal)
        computeTargetFromMode(targetRA, targetDEC);
        if (isCalibrated) {
          float elapsed = (millis() - cal_time_ms) / 1000.0;
          float drift = SIDEREAL_RATE_DEG_S * elapsed;
          float des_m_ra = cal_mount_ra + (targetRA - cal_sky_ra) + drift;
          float des_m_dec = cal_mount_dec + (targetDEC - cal_sky_dec);
          axisRA.setTargetPosition(des_m_ra, false);
          axisDEC.setTargetPosition(des_m_dec, false);
        } else {
          axisRA.setTargetPosition(targetRA, false);
          axisDEC.setTargetPosition(targetDEC, false);
        }
      }
      break;

    case ALIGN_TRACKING:
      // Handled in updateTrackingModes
      break;

    default:
      break;
  }
}

void updateTrackingModes() {
  if (!trackingActive || alignState != ALIGN_TRACKING) return;

  // Continuously update target sky from time / rate (for solar/lunar mainly)
  float newRA, newDEC;
  computeTargetFromMode(newRA, newDEC);

  targetRA = newRA;
  targetDEC = newDEC;

  if (!isCalibrated) {
    // Fallback old behavior
    axisRA.setTargetPosition(targetRA, false);
    axisDEC.setTargetPosition(targetDEC, false);
    return;
  }

  float elapsed = (millis() - cal_time_ms) / 1000.0;
  float drift = SIDEREAL_RATE_DEG_S * elapsed;
  float desired_mount_ra = cal_mount_ra + (targetRA - cal_sky_ra) + drift;
  float desired_mount_dec = cal_mount_dec + (targetDEC - cal_sky_dec);
  // Use position following for ALL modes (including sidereal) so the RA axis is driven
  // to the exact required mount angle. This matches the method that makes solar/lunar
  // reach near-zero error and ensures sidereal holds the fixed sky target precisely
  // by advancing the mount at sidereal rate via the time-dependent target.
  axisRA.setTargetPosition(desired_mount_ra, false);
  axisDEC.setTargetPosition(desired_mount_dec, false);
}

// ============================================================
// EEPROM HELPER FUNCTIONS
// ============================================================

void saveLocationToEEPROM() {
  // Save current location to EEPROM with magic number for validation
  float lat = OBS_LAT_DEG;
  float lon = OBS_LON_DEG;
  
  // Write latitude
  byte* latBytes = (byte*)&lat;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EEPROM_LAT_ADDR + i, latBytes[i]);
  }
  
  // Write longitude
  byte* lonBytes = (byte*)&lon;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EEPROM_LON_ADDR + i, lonBytes[i]);
  }
  
  // Write magic number to indicate valid data
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  
  Serial.println("STATUS:EEPROM_SAVED");
}

void loadLocationFromEEPROM() {
  // Check if EEPROM has valid data (magic number present)
  byte magic = EEPROM.read(EEPROM_MAGIC_ADDR);
  if (magic != EEPROM_MAGIC_VALUE) {
    Serial.println("STATUS:EEPROM_EMPTY");
    return;
  }
  
  // Read latitude
  float lat;
  byte* latBytes = (byte*)&lat;
  for (int i = 0; i < 4; i++) {
    latBytes[i] = EEPROM.read(EEPROM_LAT_ADDR + i);
  }
  
  // Read longitude
  float lon;
  byte* lonBytes = (byte*)&lon;
  for (int i = 0; i < 4; i++) {
    lonBytes[i] = EEPROM.read(EEPROM_LON_ADDR + i);
  }
  
  // Update current location
  OBS_LAT_DEG = lat;
  OBS_LON_DEG = lon;
  
  Serial.print("STATUS:EEPROM_LOADED,LAT:");
  Serial.print(lat, 6);
  Serial.print(",LON:");
  Serial.println(lon, 6);
}

void queryLocation() {
  // Send current location (from RAM, not necessarily from EEPROM)
  Serial.print("STATUS:LOCATION_QUERY,LAT:");
  Serial.print(OBS_LAT_DEG, 6);
  Serial.print(",LON:");
  Serial.println(OBS_LON_DEG, 6);
}

// ============================================================
// ARDUINO SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  // Removed while(!Serial) to allow Arduino to run independently of GUI
  // This enables proper reconnection when GUI quits and reopens
  delay(100); // Small delay to ensure serial port is ready

  // Configure axes
  axisRA.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
  axisRA.setAcceleration(SLEW_ACCEL_DEG_S2);

  axisDEC.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
  axisDEC.setAcceleration(SLEW_ACCEL_DEG_S2);

  // Start with drivers disabled
  axisRA.enable(false);
  axisDEC.enable(false);

  // Try to load location from EEPROM automatically on startup
  // This makes the Arduino retain its location across power cycles
  loadLocationFromEEPROM();
  
  // Default time (will be overridden by Python app)
  setTimeFromValues(2026, 7, 11, 12, 0, 0);

  // Set initial positions at home
  axisRA.setCurrentPosition(0.0f);
  axisDEC.setCurrentPosition(0.0f);

  targetRA = 0.0f;
  targetDEC = 0.0f;
  syncRA = 0.0f;
  syncDEC = 0.0f;
  siderealTargetRA = 0.0f;
  siderealTargetDEC = 0.0f;
  siderealTargetSet = false;

  Serial.print("STATUS:FIRMWARE:");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("STATUS:READY,AXIS:BOTH");
  Serial.print("STATUS:UPDATE_RATE,MS:");
  Serial.println(positionUpdateIntervalMs);
  sendPositionUpdateNow();
}

void loop() {
  unsigned long now = millis();

  // 1. Serial
  readSerialCommands();

  // 2. Advance software clock
  updateTimeFromMillis();

  // 3. Alignment state machine
  runAlignmentSequence();

  // 4. If tracking, update targets
  updateTrackingModes();

  // 5. Always run motor controllers (non-blocking)
  axisRA.update();
  axisDEC.update();

  // 6. Handle sequenced manual slew (DEC first, then RA)
  if (isManualSlewing) {
    // When DEC is close enough to its target, start RA slew
    float decErr = fabs(axisDEC.getCurrentAngle() - slewTargetDEC);
    if (!axisDEC.isMoving() || decErr < 0.8f) {
      axisRA.setMaxSpeed(MAX_SLEW_SPEED_DEG_S);
      axisRA.setAcceleration(SLEW_ACCEL_DEG_S2);
      axisRA.setTargetPosition(slewTargetRA, true);
      isManualSlewing = false;
      switchMicrosteps(TRACKING_MICROSTEPS);
      sendStatus("SLEWING", "RA");
    }
  }

  // 7. Periodic position broadcast (configurable from GUI). Call every iteration - the function
  // itself decides whether to start a new send or advance one already in progress (see its
  // comment for why this is spread across multiple loop() calls instead of done in one shot).
  updatePositionBroadcast();

  // 8. Heavy debug state dump (enabled with CMD,DEBUG,ON from the GUI / serial).
  // This prints *a lot* of variables (calibration, sky/mount computed, times, targets, etc.)
  // Useful to see exactly what the Arduino thinks after a SYNC.
  // Rate limited so it doesn't completely flood when turned on.
  if (debugVerbose && (now - lastDebugDumpMs >= 250)) {
    dumpDebugState("PERIODIC");
    lastDebugDumpMs = now;
  }

  // Small yield
  // No delay() anywhere - fully non-blocking
}
