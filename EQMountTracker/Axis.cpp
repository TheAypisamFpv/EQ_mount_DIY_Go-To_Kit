/*
  Axis.cpp - Implementation of non-blocking EQ mount axis, backed by StepGen

  See the design note at the top of Axis.h for the full history (AccelStepper polled from loop()
  -> failed ISR attempt in 1.8.12 -> this StepGen-backed version). The short version: update()
  below computes the current desired step rate (ramp math, sqrt/float, can take its time) and
  hands it to StepGen, whose own fixed-tick ISR (see StepGen.cpp) executes it autonomously,
  immune to how promptly loop() calls update() again.
*/

#include "Axis.h"
#include <math.h>

Axis::Axis(float gearRatio, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, StepGenAxis axisId, bool invertDirection, float backlashDeg)
    : _axisId(axisId),
      _stepPin(stepPin),
      _dirPin(dirPin),
      _invertDirection(invertDirection),
      _gearRatio(gearRatio),
      _targetAngle(0.0f),
      _useAcceleration(true),
      _maxSpeedDegPerSec(1.0f),
      _accelerationDegPerSec2(0.5f),
      _fineAccelDegPerSec2(5.0f),
      _stopAccelDegPerSec2(5.0f),
      _usingStopAccel(false),
      _enablePin(enablePin),
      _speedMode(false),
      _constantSpeedDegPerSec(0.0f),
      _currentSpeedStepsPerSec(0.0f),
      _lastRampCalcMicros(0),
      _rampStarted(false),
      _lastSpeedCalcMs(0),
      _lastSpeedSteps(0),
      _measuredSpeedDegPerSec(0.0f),
      _backlashDeg(backlashDeg),
      _backlashOffsetSteps(0),
      _lastMoveDirection(0)
{
    // Constructor runs during static init, before setup()/loop(). Only cheap, hardware-safe setup
    // here - StepGen's timer registers are armed separately via beginStepGen(), called explicitly
    // from setup() (mirrors where the old, reverted setupStepperTimer() call used to live).
    pinMode(_enablePin, OUTPUT);
    enable(false);  // start disabled for safety
}

void Axis::beginStepGen() {
    StepGen::begin(_axisId, _stepPin, _dirPin);
}

void Axis::enable(bool enabled) {
    digitalWrite(_enablePin, enabled ? LOW : HIGH);  // TMC / common CNC: LOW enables
}

void Axis::setCurrentPosition(float angleDeg) {
    long steps = angleToSteps(angleDeg);
    StepGen::setPosition(_axisId, steps);
    _targetAngle = angleDeg;
    _currentSpeedStepsPerSec = 0.0f;
    // A hard position reset (sync/calibration) declares "the axis IS at angleDeg right now" -
    // any backlash bookkeeping from before is about a mechanical direction history that's no
    // longer meaningful to assume either way, so start clean rather than let a stale offset
    // make the very next getCurrentAngle() call disagree with the angleDeg just set here.
    _backlashOffsetSteps = 0;
    _lastMoveDirection = 0;
}

float Axis::getCurrentAngle() {
    return stepsToAngle(StepGen::getPosition(_axisId) - _backlashOffsetSteps);
}

long Axis::getCurrentSteps() const {
    return StepGen::getPosition(_axisId);
}

float Axis::getCurrentSpeedDegPerSec() const {
    unsigned long now = millis();
    long curr = StepGen::getPosition(_axisId);

    if (_lastSpeedCalcMs != 0) {
        float dt = (now - _lastSpeedCalcMs) / 1000.0f;
        if (dt > 0.01f) {  // run calc only with sufficient dt to avoid FP precision errors (e.g. tiny dt -> 0)
            float sps = (curr - _lastSpeedSteps) / dt;
            _measuredSpeedDegPerSec = sps / _gearRatio;
            _lastSpeedSteps = curr;
            _lastSpeedCalcMs = now;
        }
        // if dt too small, keep previous value
    } else {
        _lastSpeedCalcMs = now;
        _lastSpeedSteps = curr;
        _measuredSpeedDegPerSec = 0.0f;
    }

    return _measuredSpeedDegPerSec;
}

void Axis::setTargetPosition(float targetAngleDeg, bool useAcceleration) {
    // Backlash compensation - see setBacklash()'s comment for the measurement procedure. Detects
    // a direction reversal by comparing the raw-step direction this new target implies against
    // the last commanded move's direction; only possible once a direction has actually been
    // established (_lastMoveDirection != 0) - the first move of a session has nothing to compare
    // against yet, so it's assumed already fully seated (no compensation applied).
    long currentRawSteps = StepGen::getPosition(_axisId);
    long newTargetRawSteps = angleToSteps(targetAngleDeg) + _backlashOffsetSteps;
    long distanceToGo = newTargetRawSteps - currentRawSteps;
    int8_t desiredDir = (distanceToGo > 0) ? 1 : (distanceToGo < 0) ? -1 : 0;
    if (desiredDir != 0 && _lastMoveDirection != 0 && desiredDir != _lastMoveDirection) {
        // Reversing - silently extend how far the motor is actually asked to turn by the
        // calibrated backlash amount, and grow the raw-vs-logical gap by the same amount so
        // getCurrentAngle()/isMoving() see this as "not there yet" until the real (backlash-only,
        // no real output movement) travel is done, exactly like the real mechanism behaves.
        long backlashSteps = lround(_backlashDeg * _gearRatio);
        _backlashOffsetSteps += (long)desiredDir * backlashSteps;
    }
    if (desiredDir != 0) _lastMoveDirection = desiredDir;

    _targetAngle = targetAngleDeg;
    _useAcceleration = useAcceleration;
    _speedMode = false;
    _usingStopAccel = false;
}

void Axis::commandStop(float holdAngleDeg) {
    _targetAngle = holdAngleDeg;
    _useAcceleration = false;
    _speedMode = false;
    _usingStopAccel = true;
}

void Axis::setSpeed(float speedDegPerSec) {
    _speedMode = true;
    _constantSpeedDegPerSec = speedDegPerSec;
}

void Axis::setMaxSpeed(float maxSpeedDegPerSec) {
    _maxSpeedDegPerSec = maxSpeedDegPerSec;
}

void Axis::setAcceleration(float accelDegPerSec2) {
    _accelerationDegPerSec2 = accelDegPerSec2;
}

void Axis::setFineAcceleration(float accelDegPerSec2) {
    _fineAccelDegPerSec2 = accelDegPerSec2;
}

void Axis::setStopAcceleration(float accelDegPerSec2) {
    _stopAccelDegPerSec2 = accelDegPerSec2;
}

void Axis::setGearRatio(float newGearRatio) {
    float ang = getCurrentAngle();
    _gearRatio = newGearRatio;
    long newSteps = lround(ang * _gearRatio);
    StepGen::setPosition(_axisId, newSteps);
    // Same reasoning as setCurrentPosition(): this is a hard "the raw step count now IS this
    // logical angle, exactly" reset, so any pending backlash offset (computed against the OLD
    // gear ratio's step scale) no longer means anything meaningful and would just make the next
    // getCurrentAngle() disagree with ang.
    _backlashOffsetSteps = 0;
    _lastMoveDirection = 0;
}

void Axis::setBacklash(float backlashDeg) {
    _backlashDeg = backlashDeg;
}

void Axis::resetSpeedCalculation() {
    long pos = StepGen::getPosition(_axisId);
    _lastSpeedCalcMs = millis();
    _lastSpeedSteps = pos;
    _measuredSpeedDegPerSec = 0.0f;
}

void Axis::update() {
    // Called from loop() (see the .ino's "5. Always run motor controllers" step). This decides
    // the CURRENT step rate only - it does not generate step pulses itself (StepGen's ISR does
    // that independently, at whatever rate this last set). Safe to take 100+us occasionally.
    unsigned long now = micros();
    if (!_rampStarted) {
        _lastRampCalcMicros = now;
        _rampStarted = true;
        return;
    }
    float dt = (now - _lastRampCalcMicros) / 1000000.0f;
    _lastRampCalcMicros = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;  // guard against overflow/first-call/long-gap weirdness

    if (_speedMode) {
        // Constant-speed tracking: no ramp, go straight to the requested rate.
        _currentSpeedStepsPerSec = _constantSpeedDegPerSec * _gearRatio;
    } else {
        // + _backlashOffsetSteps: see setTargetPosition()'s comment - keeps this consistent with
        // whatever backlash take-up travel was silently added there.
        long targetSteps = angleToSteps(_targetAngle) + _backlashOffsetSteps;
        long currentSteps = StepGen::getPosition(_axisId);
        long distanceToGo = targetSteps - currentSteps;

        float maxSpeedSteps = _maxSpeedDegPerSec * _gearRatio;
        // useAcceleration=false used to mean an effectively-infinite accel (1000000) here - an
        // instantaneous commanded speed jump that no real motor can actually follow, which is
        // exactly what produced an audible jolt on every tracking correction and every stop even
        // though position/timing were both still correct (see the 1.8.24 changelog entry). Now
        // uses _fineAccelDegPerSec2 (setFineAcceleration()) for ordinary tracking corrections, or
        // _stopAccelDegPerSec2 (setStopAcceleration()) specifically while commandStop()'s
        // override is active - a separate, faster deceleration for stopping, tuned independently
        // of tracking-correction smoothness.
        float fineOrStopAccel = _usingStopAccel ? _stopAccelDegPerSec2 : _fineAccelDegPerSec2;
        float accelSteps = (_useAcceleration ? _accelerationDegPerSec2 : fineOrStopAccel) * _gearRatio;
        if (accelSteps < 1.0f) accelSteps = 1.0f;

        float desiredSpeed;
        if (distanceToGo == 0) {
            desiredSpeed = 0.0f;
        } else {
            // Same trapezoidal kinematics as before: cap speed so there's still room to decelerate
            // to zero exactly at the target (v = sqrt(2 * a * distance)), clamped to max speed.
            float dist = fabs((float)distanceToGo);
            float brakingSpeed = sqrt(2.0f * accelSteps * dist);
            float cap = (maxSpeedSteps < brakingSpeed) ? maxSpeedSteps : brakingSpeed;
            desiredSpeed = (distanceToGo > 0) ? cap : -cap;
        }

        float maxDelta = accelSteps * dt;
        float diff = desiredSpeed - _currentSpeedStepsPerSec;
        if (diff > maxDelta) diff = maxDelta;
        if (diff < -maxDelta) diff = -maxDelta;
        _currentSpeedStepsPerSec += diff;

        if (distanceToGo == 0) _currentSpeedStepsPerSec = 0.0f;
    }

    bool positionIncreasing = _currentSpeedStepsPerSec >= 0.0f;
    bool dirPinHigh = _invertDirection ? !positionIncreasing : positionIncreasing;
    StepGen::setRate(_axisId, fabs(_currentSpeedStepsPerSec), positionIncreasing, dirPinHigh);
}

bool Axis::isMoving() {
    if (_speedMode) {
        return fabs(_constantSpeedDegPerSec) > 0.0001f;
    }
    long targetSteps = angleToSteps(_targetAngle) + _backlashOffsetSteps;  // see update()'s comment
    long currentSteps = StepGen::getPosition(_axisId);
    return labs(targetSteps - currentSteps) > 4;
}

long Axis::angleToSteps(float angleDeg) const {
    return lround(angleDeg * _gearRatio);
}

float Axis::stepsToAngle(long steps) const {
    return (float)steps / _gearRatio;
}
