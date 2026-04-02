#ifndef SwerveModule_h
#define SwerveModule_h
#include "Arduino.h"
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
class SwerveModule
{
public:
    /**
     * Construct a swerve module controller.
     *
     * @param driveServoPort Port for the drive motor controller.
     * @param driveMotorInversion True to invert the drive motor direction.
     * @param turnServoPort Port for the steering servo.
     * @param turnServoInversion True to invert steering direction.
     * @param angleoffset Steering zero offset in degrees.
     * @param wrapendpoint Steering wrap limit in degrees.
     * @param drivegearratio Drive gear ratio from motor to wheel.
     * @param turngearratio Steering gear ratio from servo to module.
     * @param brakeMode True to enable drive motor braking.
     */
    SwerveModule(uint8_t driveServoPort, bool driveMotorInversion = false, uint8_t turnServoPort, bool turnServoInversion = false, float angleoffset, float wrapendpoint, float drivegearratio, float turngearratio, bool brakeMode = false);

    /**
     * Drive the module using a field-relative target angle.
     *
     * @param targetAngle Desired field-relative steering angle in degrees.
     * @param driveSpeed Desired non-directional drive speed command.
     */
    void driveModule(float targetAngle, float driveSpeed);

    /**
     * Drive the module using a robot-relative target angle.
     *
     * @param targetAngle Desired robot-relative steering angle in degrees.
     * @param driveSpeed Desired directional drive speed command.
     */
    void directDriveModule(float targetAngle, float driveSpeed);

    /**
     * Get the current robot-relative module angle.
     *
     * @return Current module steering angle in degrees.
     */
    float getModuleAngle();

    /**
     * Get the current module drive velocity.
     *
     * @return Current wheel speed estimate.
     */
    float getModuleVelocity();

    /**
     * Set brake mode for the drive motor.
     *
     * @param brake True to enable braking, false to coast.
     */
    void setBrakeMode(bool brake);

    /**
     * Get direct access to the drive motor object.
     *
     * @return Pointer to the internal drive motor (never nullptr).
     */
    NoU_Motor *getDriveMotor();

    /**
     * Get direct access to the steering servo object.
     *
     * @return Pointer to the internal steering servo (never nullptr).
     */
    NoU_Servo *getTurnServo();

    /**
     * Get a copy of this module object.
     *
     * @return Copy of the current SwerveModule instance.
     */
    SwerveModule getModule();

private:
    NoU_Motor driveMotor;
    NoU_Servo turnServo;
    Encoder driveEncoder;
    float angleOffset;
    float wrapEndpoint;
    float driveGearRatio;
    float turnGearRatio;
    float currentSpeed;
    float currentAngle;
    bool turnInversion;
};
#endif