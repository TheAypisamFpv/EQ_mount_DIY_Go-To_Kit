#include "StepGen.h"
#include <avr/interrupt.h>

namespace {

  // 16MHz / prescaler 8 = 2MHz timer clock (0.5us/count). OCR=99 -> 100 counts -> exactly 50us.
  const uint16_t TICK_OCR = 99;
  const double TICK_PERIOD_S = 50.0e-6;

  // Fixed-point scale for the accumulator: rateFixed/ACCUM_SCALE = steps per tick.
  const uint32_t ACCUM_SCALE = 65536UL;

  // Must stay comfortably below 1 step/tick (20000 steps/sec at a 50us tick) so the ISR's single
  // "if (accum >= ACCUM_SCALE) accum -= ACCUM_SCALE" can never need to fire more than once per
  // tick. Our fastest real requirement is ~5778 steps/sec (MAX_SLEW_SPEED_DEG_S=10 at 8
  // microsteps, 130:1 RA ratio) - this leaves over 3x headroom.
  const float MAX_STEPS_PER_SEC = 18000.0f;

  struct AxisStepState {
    volatile uint32_t accum;
    volatile uint32_t rateFixed;
    volatile int8_t direction;   // +1 or -1, for position bookkeeping only
    volatile bool pendingLow;
    volatile long position;
    uint8_t stepPin;
    uint8_t dirPin;
    bool lastDirPinHigh;         // only touched with interrupts disabled - see setRate()
  };

  AxisStepState g_ra;
  AxisStepState g_dec;

  inline AxisStepState &stateFor(StepGenAxis axis) {
    return (axis == STEPGEN_RA) ? g_ra : g_dec;
  }

  // Called from both ISRs below - fixed, bounded work only: no float, no division, at most one
  // digitalWrite(). This is the property the 1.8.12 attempt violated (it ran AccelStepper's full
  // ramp computation, which had unbounded cost, inside the ISR instead).
  inline void stepTick(AxisStepState &s) {
    if (s.pendingLow) {
      digitalWrite(s.stepPin, LOW);
      s.pendingLow = false;
    }
    s.accum += s.rateFixed;
    if (s.accum >= ACCUM_SCALE) {
      s.accum -= ACCUM_SCALE;
      digitalWrite(s.stepPin, HIGH);
      s.pendingLow = true;
      s.position += s.direction;
    }
  }

} // namespace

ISR(TIMER1_COMPA_vect) { stepTick(g_ra); }
ISR(TIMER3_COMPA_vect) { stepTick(g_dec); }

namespace StepGen {

  void begin(StepGenAxis axis, uint8_t stepPin, uint8_t dirPin) {
    AxisStepState &s = stateFor(axis);
    s.accum = 0;
    s.rateFixed = 0;
    s.direction = 1;
    s.pendingLow = false;
    s.position = 0;
    s.stepPin = stepPin;
    s.dirPin = dirPin;
    s.lastDirPinHigh = false;

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, LOW);

    noInterrupts();
    if (axis == STEPGEN_RA) {
      TCCR1A = 0;
      TCCR1B = (1 << WGM12) | (1 << CS11);  // CTC (OCR1A=TOP), prescaler 8
      OCR1A = TICK_OCR;
      TCNT1 = 0;
      TIMSK1 |= (1 << OCIE1A);
    } else {
      TCCR3A = 0;
      TCCR3B = (1 << WGM32) | (1 << CS31);  // CTC (OCR3A=TOP), prescaler 8
      OCR3A = TICK_OCR;
      TCNT3 = 0;
      TIMSK3 |= (1 << OCIE3A);
    }
    interrupts();
  }

  void setRate(StepGenAxis axis, float stepsPerSec, bool positionIncreasing, bool dirPinHigh) {
    AxisStepState &s = stateFor(axis);
    if (stepsPerSec < 0.0f) stepsPerSec = 0.0f;
    if (stepsPerSec > MAX_STEPS_PER_SEC) stepsPerSec = MAX_STEPS_PER_SEC;
    uint32_t rf = (uint32_t)((double)stepsPerSec * TICK_PERIOD_S * (double)ACCUM_SCALE + 0.5);

    // The DIR pin write must happen atomically together with rateFixed/direction, not before it -
    // otherwise there's a window (while interrupts are still enabled, mid-digitalWrite) where the
    // physical DIR level has already changed but the ISR is still using the OLD direction/rate,
    // so a step fired in that window moves the motor one way while position bookkeeping records
    // the other. Found via user report: audible clicks even at rest / near-zero speed, and
    // tracking precision regressing - exactly what a mis-attributed step direction would cause.
    // Only writing the pin when it actually changes also avoids paying this cost on every call,
    // since update() calls this on nearly every loop() iteration.
    noInterrupts();
    if (dirPinHigh != s.lastDirPinHigh) {
      digitalWrite(s.dirPin, dirPinHigh ? HIGH : LOW);
      s.lastDirPinHigh = dirPinHigh;
    }
    s.rateFixed = rf;
    s.direction = positionIncreasing ? 1 : -1;
    interrupts();
  }

  void stop(StepGenAxis axis) {
    AxisStepState &s = stateFor(axis);
    noInterrupts();
    s.rateFixed = 0;
    interrupts();
  }

  long getPosition(StepGenAxis axis) {
    AxisStepState &s = stateFor(axis);
    noInterrupts();
    long p = s.position;
    interrupts();
    return p;
  }

  void setPosition(StepGenAxis axis, long steps) {
    AxisStepState &s = stateFor(axis);
    noInterrupts();
    s.position = steps;
    s.accum = 0;
    interrupts();
  }

}
