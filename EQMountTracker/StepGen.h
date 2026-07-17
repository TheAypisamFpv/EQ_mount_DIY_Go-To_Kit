/*
  StepGen.h - Fixed-tick, interrupt-driven step pulse generator for exactly two axes (RA on
  Timer1, DEC on Timer3 - the Mega 2560's two free 16-bit timers; Timer0 is owned by the Arduino
  core for millis()/delay() and is never touched here).

  This exists to make step pulse TIMING fully immune to whatever loop() is doing (Serial parsing,
  position broadcasts, EEPROM writes, etc.) - see the design note at the top of Axis.h for the
  full history of why (1.8.12 attempted this and failed badly; this is the corrected version).

  The key rule that makes this safe, unlike the 1.8.12 attempt: the ISRs below do ONLY bounded,
  fixed-cost integer work every tick (~50us) - an accumulator add, a compare, at most one
  digitalWrite(). They never do the acceleration ramp math (sqrt, float division) that caused the
  earlier ISR to occasionally run long enough to stall loop() for seconds. That ramp math still
  happens, but in Axis::update(), called from loop() - it's free to take its time there, because
  all it does is decide the CURRENT step rate; the ISR just keeps stepping autonomously at
  whatever rate it was last given, regardless of how long loop() takes to come back and update it.

  Step generation is a fixed-period-timer + accumulator design (a digital differential analyzer,
  the same technique GRBL and most from-scratch AVR step generators use) rather than reprogramming
  the timer's compare register per step: the required step rate ranges from ~5778 steps/sec at
  MAX_SLEW_SPEED_DEG_S=10 down to ~2.4 steps/sec at sidereal tracking - too wide a dynamic range
  to represent as a single reprogrammed interval without switching prescalers. A fixed 50us tick
  with an accumulator handles that whole range with one timer configuration, at the cost of the
  ISR firing (cheaply) even when no step is due that tick.
*/

#ifndef STEPGEN_H
#define STEPGEN_H

#include <Arduino.h>

enum StepGenAxis { STEPGEN_RA, STEPGEN_DEC };

namespace StepGen {

  // Configures the hardware timer for this axis (Timer1 for STEPGEN_RA, Timer3 for STEPGEN_DEC)
  // and readies stepPin/dirPin as outputs. Call once from setup(), not from a global constructor
  // (mirrors where the old, reverted setupStepperTimer() call used to live).
  void begin(StepGenAxis axis, uint8_t stepPin, uint8_t dirPin);

  // Sets the current step rate. stepsPerSec is an unsigned magnitude (clamped internally to a
  // safe maximum well below the 1-step-per-tick limit this design requires - see StepGen.cpp).
  // positionIncreasing selects the sign used for the authoritative step position counter
  // (independent of wiring). dirPinHigh is the literal level written to the DIR pin - the caller
  // (Axis) is responsible for folding any direction-invert setting into this, since StepGen has
  // no notion of inversion itself. Safe to call from loop() at any time.
  //
  // Implementation note (1.8.23 fix): the DIR pin write and the rateFixed/direction update this
  // performs internally MUST happen as one atomic (interrupts-disabled) operation - see the
  // comment in StepGen.cpp's setRate(). Writing the pin outside that critical section briefly
  // let the ISR observe a physical DIR level that didn't match its own direction/rate state yet,
  // causing occasional wrong-direction steps (audible clicks at rest, small tracking errors).
  void setRate(StepGenAxis axis, float stepsPerSec, bool positionIncreasing, bool dirPinHigh);

  // Zeroes the step rate immediately. Takes effect within one tick (~50us) - used for emergency
  // stop, replacing the old hope-based reliance on loop() promptly seeing a stop command.
  void stop(StepGenAxis axis);

  // Authoritative step position, maintained by the ISR itself. Reads/writes are guarded against
  // the ISR internally (a plain long isn't atomic on an 8-bit AVR).
  long getPosition(StepGenAxis axis);
  void setPosition(StepGenAxis axis, long steps);

}

#endif // STEPGEN_H
