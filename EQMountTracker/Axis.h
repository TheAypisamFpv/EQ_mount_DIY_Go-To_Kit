
/*
  Axis.h - Reusable non-blocking axis controller for EQ mount, backed by StepGen (interrupt-
  driven step pulse generation)
  Part of DIY EQ Mount Go-To Kit

  Provides sky-angle based positioning with optional acceleration profiles.

  History of how stepping is generated, since this has gone through three designs:
   - Originally (through 1.8.16, minus a brief attempt below): AccelStepper, polled from loop()
     via update() -> _stepper.run()/runSpeed(). Simple, but step pulse timing was only as precise
     as how promptly loop() got back to it - any other work in loop() (a Serial.print(float), a
     command parse) delayed the next step and showed up as an audible twitch.
   - 1.8.12-1.8.16: tried moving that same AccelStepper::run() call into a fixed-period Timer1
     ISR, hoping to make stepping immune to loop() delays. Reverted in 1.8.17 after bench testing
     found a worse regression: AccelStepper::run() doesn't just toggle a step pin, it also
     recomputes the acceleration ramp (sqrt/float division) on every call, and that computation's
     cost is NOT bounded. Inside a fixed 100us ISR, whenever a recompute took longer than 100us,
     the ISR's own pending-interrupt flag stayed set and refired immediately - giving the ISR
     ~100% of the CPU for as long as that persisted, observed as multi-second total loop() stalls
     during active slews (this also explained why a dedicated emergency-stop panic byte didn't
     help - the code reading it wasn't running either, since loop() was starved).
   - Current (this version): StepGen (see StepGen.h) generates the actual step pulses from a
     fixed 50us-tick hardware timer ISR that does ONLY bounded integer work (an accumulator
     add/compare, at most one digitalWrite()) - never the ramp math. The ramp math itself lives
     in Axis::update() below, called from loop() same as before, but now it's harmless for it to
     take 100+us occasionally: it only decides the CURRENT step rate, which StepGen keeps
     executing autonomously between calls, immune to whatever else loop() is doing. This is the
     "correct" version of what 1.8.12 was trying to do - see StepGen.h's header comment and
     TODO.md for the full design reasoning (modeled on GRBL's AVR stepper driver architecture).
*/

#ifndef AXIS_H
#define AXIS_H

#include <Arduino.h>
#include "StepGen.h"

class Axis {
public:
    // gearRatio = total motor steps per degree of movement on the sky (after all gearing and
    // microstepping). Example for this setup: 200 * 8 * 130 / 360
    // axisId selects which dedicated hardware timer this axis uses for step generation (see
    // StepGen.h) - must be unique per Axis instance (one STEPGEN_RA, one STEPGEN_DEC).
    // invertDirection flips which physical DIR pin level corresponds to a positive step, WITHOUT
    // touching any angle/step/gear-ratio math or the step position counter's sign - it only
    // changes what gets written to the DIR pin (folded in when calling StepGen::setRate()), so
    // existing position/calibration/tracking math still means exactly the same thing in sky
    // coordinates, just with the motor spinning the other way to get there.
    // backlashDeg = mechanical slack (in this axis's own mount-angle degrees, same scale as every
    // other angle here) that has to be taken up, silently and automatically, whenever a commanded
    // move reverses direction relative to the last one - see setBacklash()'s comment for how to
    // measure it and setTargetPosition()'s for how it's applied.
    Axis(float gearRatio, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, StepGenAxis axisId, bool invertDirection = false, float backlashDeg = 0.0f);

    // Arms this axis's hardware timer for step generation. Call once from setup(), not from a
    // global constructor (mirrors where the old, reverted setupStepperTimer() call used to live).
    void beginStepGen();

    // Set the current sky angle (e.g. from sync). Updates the step position mapping.
    void setCurrentPosition(float angleDeg);

    // Command a move to a target sky angle.
    // If useAcceleration=true: uses the configured accel/decel profile (setAcceleration(), for
    // slews/alignment).
    // If useAcceleration=false: uses the configured FINE acceleration profile
    // (setFineAcceleration(), for tracking-rate corrections) - fast enough for a prompt response,
    // but a real finite ramp, not an instantaneous speed jump (see the 1.8.24 changelog entry in
    // EQMountTracker.ino for why that mattered). Calling this clears any pending commandStop()
    // acceleration override (see below) - a normal slew/tracking command always means the stop
    // override no longer applies.
    void setTargetPosition(float targetAngleDeg, bool useAcceleration);

    // Commands the axis to hold at holdAngleDeg (normally its current position) using
    // setStopAcceleration()'s deceleration instead of the fine-tracking-correction one - a
    // dedicated, faster ramp for stopping specifically, separate from FINE_ACCEL_DEG_S2 so tuning
    // stop responsiveness doesn't also affect ordinary tracking-correction smoothness. The
    // override is "sticky" (stays in effect update() after update() until the deceleration
    // finishes) but gets cleared automatically the next time setTargetPosition() is called for a
    // real slew/tracking move.
    void commandStop(float holdAngleDeg);

    // Set constant speed in sky deg/sec (no ramp). Use for pure rate tracking if preferred over position updates.
    void setSpeed(float speedDegPerSec);

    // Current sky angle computed from actual step position
    float getCurrentAngle();

    // Raw step position - this is the position relative to the mount/drive
    long getCurrentSteps() const;

    // Current speed in sky degrees per second (can be negative)
    float getCurrentSpeedDegPerSec() const;

    // Must be called frequently from main loop - runs the speed ramp calculation (see the class
    // comment above for why this is safe to take its time; it no longer generates step pulses
    // itself, StepGen's ISR does that independently).
    void update();

    // Enable/disable the driver (enablePin logic: LOW = enabled for most drivers)
    void enable(bool enabled);

    // Is the axis currently moving significantly?
    bool isMoving();

    // Accessors for configuration
    void setMaxSpeed(float maxSpeedDegPerSec);
    void setAcceleration(float accelDegPerSec2);

    // Acceleration used when setTargetPosition() is called with useAcceleration=false - see the
    // comment there. Not literally instantaneous, but still fast/responsive.
    void setFineAcceleration(float accelDegPerSec2);

    // Acceleration used by commandStop() - see its comment.
    void setStopAcceleration(float accelDegPerSec2);

    // Mechanical backlash compensation amount, in this axis's own mount-angle degrees. To
    // measure it on real hardware: move the axis in one direction until it's fully mechanically
    // engaged (e.g. via the GUI's offset nudge, in small increments), then start moving it back
    // the OTHER way in small increments (e.g. 0.01 deg at a time) and count how many degrees of
    // commanded movement it takes before the telescope actually visibly starts moving again -
    // that count IS the backlash amount for that axis. Whenever setTargetPosition() detects the
    // requested move reverses direction from the previous one, it silently adds this many extra
    // degrees of travel in the new direction first (tracked internally, transparent to the
    // caller - see _backlashOffsetSteps) before the real, intended movement actually begins.
    void setBacklash(float backlashDeg);

    float getTargetAngle() const { return _targetAngle; }

    // Change gear ratio (e.g. for different microstepping during slews vs tracking).
    // Preserves the current physical angle by rescaling the internal step count.
    void setGearRatio(float newGearRatio);

    void resetSpeedCalculation();

private:
    StepGenAxis _axisId;
    uint8_t _stepPin, _dirPin;
    bool _invertDirection;

    float _gearRatio;           // steps per sky degree
    float _targetAngle;
    bool _useAcceleration;
    float _maxSpeedDegPerSec;
    float _accelerationDegPerSec2;
    float _fineAccelDegPerSec2; // used instead of _accelerationDegPerSec2 when useAcceleration=false
    float _stopAccelDegPerSec2; // used instead of _fineAccelDegPerSec2 while _usingStopAccel is set
    bool _usingStopAccel;       // set by commandStop(), cleared by the next setTargetPosition()
    uint8_t _enablePin;
    bool _speedMode;            // if true, use constant-speed (no ramp) mode
    float _constantSpeedDegPerSec;

    // Ramp state (see update())
    float _currentSpeedStepsPerSec;
    unsigned long _lastRampCalcMicros;
    bool _rampStarted;

    // For actual speed measurement (to report real movement, not just commanded)
    mutable unsigned long _lastSpeedCalcMs;
    mutable long _lastSpeedSteps;
    mutable float _measuredSpeedDegPerSec;

    // Backlash compensation state - see setBacklash()'s comment for the measurement procedure
    // and setTargetPosition()'s for how these are used.
    float _backlashDeg;
    // Accumulated gap (raw hardware steps) between what StepGen has actually been told to step
    // through and what the LOGICAL angle (_targetAngle, getCurrentAngle()) reports - grows by
    // +-(backlash-in-steps) every time a direction reversal is detected. getCurrentAngle(),
    // isMoving(), and update()'s target-steps calc all consistently subtract/add this so a
    // reversal's dead-zone travel is silently absorbed rather than misread as real logical
    // movement (which hasn't happened yet - that's the whole point of backlash).
    long _backlashOffsetSteps;
    // Direction (+1/-1) of the most recently COMMANDED move (not necessarily completed) -
    // 0 means "not yet established" (nothing to compare a reversal against yet, e.g. right after
    // startup/a fresh sync), in which case no compensation is applied since there's no way to
    // know whether the mechanism happens to already be seated in the requested direction or not.
    int8_t _lastMoveDirection;

    long angleToSteps(float angleDeg) const;
    float stepsToAngle(long steps) const;
};

#endif // AXIS_H
