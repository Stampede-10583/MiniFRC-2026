#include <array>
#include "SwerveDrive.h"
#include "Arduino.h"
#include "SwerveModule.h"
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"

SwerveDrive::SwerveDrive(
    uint8_t driveMotorPorts[4],
    std::array<bool, 4> driveMotorInversions,
    std::array<float, 4> driveGearRatios,
    uint8_t turnServoPorts[4],
    std::array<float, 4> turnServoOffsets,
    std::array<bool, 4> turnServoInversions,
    std::array<float, 4> turnServoGearRatios,
    std::array<float, 4> turnServoDeadspots,
    float imuAngularScale,
    float maxSpeed)
    : imuAngularScale(imuAngularScale),
      maxSpeed(maxSpeed), modules{
                              SwerveModule(driveMotorPorts[0], driveMotorInversions[0], turnServoPorts[0], turnServoOffsets[0], turnServoInversions[0], turnServoDeadspots[0]),
                              SwerveModule(driveMotorPorts[1], driveMotorInversions[1], turnServoPorts[1], turnServoOffsets[1], turnServoInversions[1], turnServoDeadspots[1]),
                              SwerveModule(driveMotorPorts[2], driveMotorInversions[2], turnServoPorts[2], turnServoOffsets[2], turnServoInversions[2], turnServoDeadspots[2]),
                              SwerveModule(driveMotorPorts[3], driveMotorInversions[3], turnServoPorts[3], turnServoOffsets[3], turnServoInversions[3], turnServoDeadspots[3])}
{
}

void SwerveDrive::drive(float targetAngle, float driveSpeed)
{
    std::array<float, 4> targetAngles = {targetAngle, targetAngle, targetAngle, targetAngle};
    std::array<float, 4> driveSpeeds = {driveSpeed, driveSpeed, driveSpeed, driveSpeed};
    for (int i = 0; i < 4; i++)
    {
        modules[i].driveModule(targetAngles[i], driveSpeeds[i]);
    }
}
void SwerveDrive::driveRobotOriented(float targetAngle, float driveSpeed)
{

    // TODO: Robot-oriented drive implementation.
}

void SwerveDrive::setModuleStates(float targetAngles[4], float driveSpeeds[4])
{
    for (int i = 0; i < 4; i++)
    {
        modules[i].driveModule(targetAngles[i], driveSpeeds[i]);
    }
    // TODO: Apply per-module angle/speed states.
}

void SwerveDrive::driveToPoint(float x, float y, float robotAngle, float maxSpeed)
{
    // TODO: Path/point drive helper.
}

void SwerveDrive::setBrakeMode(bool brake)
{
    for (int i = 0; i < 4; i++)
    {
        modules[i].setBrakeMode(brake);
    }
    // TODO: Forward brake mode to each module.
}

void SwerveDrive::setMaxSpeed(float maxSpeed)
{
    this->maxSpeed = maxSpeed;
}

void SwerveDrive::setIMUScale(float imuAngularScale)
{
    this->imuAngularScale = imuAngularScale;
}

std::array<float, 3> SwerveDrive::getOdeometry()
{
    // TODO: Return computed odometry (x, y, heading).
    return {0.0f, 0.0f, 0.0f};
}

void SwerveDrive::resetOdeometry(float x, float y, float angle)
{

    // TODO: Reset odometry state.
}

void SwerveDrive::swerveDrivePeriodic()
{
    // TODO: Periodic update for sensors/odometry/module control.
}

SwerveModule *SwerveDrive::getModule(int index)
{
    if (index < 0 || index >= 4)
    {
        return nullptr;
    }
    return &modules[index];
}

SwerveDrive SwerveDrive::getDrive()
{
    return *this;
}