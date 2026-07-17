
/*
  Axis.h - Reusable non-blocking axis controller for EQ mount using AccelStepper
  Part of DIY EQ Mount Go-To Kit

  Provides sky-angle based positioning with optional acceleration profiles.
*/

#ifndef AXIS_H
#define AXIS_H

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Arduino.h>

class Axis {
public:
    // gearRatio = total motor steps per degree of movement on the sky (after all gearing and microstepping)
    // Example for your setup: 200 * 256 * 130 / 360
    Axis(float gearRatio, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin);

    // Set the current sky angle (e.g. from sync). Updates stepper position mapping.
    void setCurrentPosition(float angleDeg);

    // Command a move to a target sky angle.
    // If useAcceleration=true: uses configured accel/decel profile (for slews/alignment)
    // If useAcceleration=false: uses near-instant response (high accel) suitable for smooth tracking updates
    void setTargetPosition(float targetAngleDeg, bool useAcceleration);

    // Set constant speed in sky deg/sec. Use for pure rate tracking if preferred over position updates.
    void setSpeed(float speedDegPerSec);

    // Current sky angle computed from actual stepper position
    float getCurrentAngle();

    // Raw stepper position (steps) - this is the position relative to the mount/drive
    long getCurrentSteps() const;

    // Current speed in sky degrees per second (can be negative)
    float getCurrentSpeedDegPerSec() const;

    // Must be called frequently from main loop
    void update();

    // Enable/disable the driver (enablePin logic: LOW = enabled for most drivers)
    void enable(bool enabled);

    // Is the axis currently moving significantly?
    bool isMoving();

    // Accessors for configuration
    void setMaxSpeed(float maxSpeedDegPerSec);
    void setAcceleration(float accelDegPerSec2);
    float getTargetAngle() const { return _targetAngle; }

    // Change gear ratio (e.g. for different microstepping during slews vs tracking).
    // Preserves the current physical angle by rescaling the internal step count.
    void setGearRatio(float newGearRatio);

    void resetSpeedCalculation();

private:
    AccelStepper _stepper;
    float _gearRatio;           // steps per sky degree
    float _targetAngle;
    bool _useAcceleration;
    float _maxSpeedDegPerSec;
    float _accelerationDegPerSec2;
    uint8_t _enablePin;
    bool _speedMode;            // if true, use runSpeed instead of position moveTo

    // For actual speed measurement (to report real movement, not just commanded)
    mutable unsigned long _lastSpeedCalcMs;
    mutable long _lastSpeedSteps;
    mutable float _measuredSpeedDegPerSec;

    long angleToSteps(float angleDeg) const;
    float stepsToAngle(long steps) const;
    void applyTargetToStepper();
};

#endif // AXIS_H
