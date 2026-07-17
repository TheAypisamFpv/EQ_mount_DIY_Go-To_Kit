/*
  Axis.cpp - Implementation of non-blocking EQ mount axis
*/

#include "Axis.h"

Axis::Axis(float gearRatio, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin)
    : _stepper(AccelStepper::DRIVER, stepPin, dirPin),
      _gearRatio(gearRatio),
      _targetAngle(0.0),
      _useAcceleration(true),
      _maxSpeedDegPerSec(1.0),
      _accelerationDegPerSec2(0.5),
      _enablePin(enablePin),
      _speedMode(false),
      _lastSpeedCalcMs(0),
      _lastSpeedSteps(0),
      _measuredSpeedDegPerSec(0.0f)
{
    pinMode(_enablePin, OUTPUT);
    enable(false);  // start disabled for safety

    _stepper.setMaxSpeed(_maxSpeedDegPerSec * _gearRatio);
    _stepper.setAcceleration(_accelerationDegPerSec2 * _gearRatio);
    _stepper.setCurrentPosition(0);

    _lastSpeedCalcMs = millis();
    _lastSpeedSteps = 0;
    _measuredSpeedDegPerSec = 0.0f;
}

void Axis::enable(bool enabled) {
    digitalWrite(_enablePin, enabled ? LOW : HIGH);  // TMC / common CNC: LOW enables
}

void Axis::setCurrentPosition(float angleDeg) {
    long steps = angleToSteps(angleDeg);
    _stepper.setCurrentPosition(steps);
    _targetAngle = angleDeg;
}

float Axis::getCurrentAngle() {
    return stepsToAngle(_stepper.currentPosition());
}

long Axis::getCurrentSteps() const {
    return _stepper.currentPosition();
}

float Axis::getCurrentSpeedDegPerSec() const {
    unsigned long now = millis();
    long curr = _stepper.currentPosition();

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
    _targetAngle = targetAngleDeg;
    _useAcceleration = useAcceleration;
    _speedMode = false;
    applyTargetToStepper();
}

void Axis::setSpeed(float speedDegPerSec) {
    _speedMode = true;
    float stepsPerSec = speedDegPerSec * _gearRatio;
    _stepper.setSpeed(stepsPerSec);
}

void Axis::setMaxSpeed(float maxSpeedDegPerSec) {
    _maxSpeedDegPerSec = maxSpeedDegPerSec;
    _stepper.setMaxSpeed(maxSpeedDegPerSec * _gearRatio);
}

void Axis::setAcceleration(float accelDegPerSec2) {
    _accelerationDegPerSec2 = accelDegPerSec2;
    _stepper.setAcceleration(accelDegPerSec2 * _gearRatio);
}

void Axis::setGearRatio(float newGearRatio) {
  float ang = getCurrentAngle();
  _gearRatio = newGearRatio;
  long newSteps = lround(ang * _gearRatio);
  _stepper.setCurrentPosition(newSteps);
  // Re-apply current speed/accel settings with new gear
  _stepper.setMaxSpeed(_maxSpeedDegPerSec * _gearRatio);
  _stepper.setAcceleration(_accelerationDegPerSec2 * _gearRatio);
}

void Axis::resetSpeedCalculation() {
  _lastSpeedCalcMs = millis();
  _lastSpeedSteps = _stepper.currentPosition();
  _measuredSpeedDegPerSec = 0.0f;
}

void Axis::update() {
    if (_speedMode) {
        _stepper.runSpeed();
    } else {
        _stepper.run();
    }
    // Speed calc moved to getCurrentSpeedDegPerSec() so it only runs when needed (e.g. on POS send),
    // not every loop iteration. This avoids FP issues with tiny dt.
}

bool Axis::isMoving() {
    // Consider moving if distance to target is more than a few steps
    long dist = _stepper.distanceToGo();
    return abs(dist) > 4;
}

long Axis::angleToSteps(float angleDeg) const {
    return lround(angleDeg * _gearRatio);
}

float Axis::stepsToAngle(long steps) const {
    return (float)steps / _gearRatio;
}

void Axis::applyTargetToStepper() {
    long targetSteps = angleToSteps(_targetAngle);

    if (_useAcceleration) {
        _stepper.setAcceleration(_accelerationDegPerSec2 * _gearRatio);
        _stepper.setMaxSpeed(_maxSpeedDegPerSec * _gearRatio);
    } else {
        // Tracking / fine following: very high accel so it follows target with minimal lag, no noticeable ramp
        _stepper.setAcceleration(1000000.0f);
        _stepper.setMaxSpeed(_maxSpeedDegPerSec * _gearRatio);
    }

    _stepper.moveTo(targetSteps);
}
