#include <array>
#include "MiniSwerveLib.h"
#include "Arduino.h"
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
    std::array<uint16_t, 2> servoConfig,
    float imuAngularScale,
    float maxSpeed)
    : imuAngularScale(imuAngularScale),
      maxSpeed(maxSpeed), modules{ 
                              SwerveModule(driveMotorPorts[0], driveMotorInversions[0], turnServoPorts[0], turnServoInversions[0], turnServoOffsets[0], turnServoDeadspots[0], driveGearRatios[0], turnServoGearRatios[0], servoConfig),
                              SwerveModule(driveMotorPorts[1], driveMotorInversions[1], turnServoPorts[1], turnServoInversions[1], turnServoOffsets[1], turnServoDeadspots[1], driveGearRatios[1], turnServoGearRatios[1], servoConfig),
                              SwerveModule(driveMotorPorts[2], driveMotorInversions[2], turnServoPorts[2], turnServoInversions[2], turnServoOffsets[2], turnServoDeadspots[2], driveGearRatios[2], turnServoGearRatios[2], servoConfig),
                              SwerveModule(driveMotorPorts[3], driveMotorInversions[3], turnServoPorts[3], turnServoInversions[3], turnServoOffsets[3], turnServoDeadspots[3], driveGearRatios[3], turnServoGearRatios[3], servoConfig)}
{
}
std::vector<float> SwerveDrive::getTargetVelocities(float xJoystick, float yJoystick, float rotationJoystick, float scale, bool robotOriented)
{

}
void SwerveDrive::drive(std::vector<float> velocities, float driveSpeed)
{
    std::array<float, 4> targetAngles = {velocities[2], velocities[2], velocities[2], velocities[2]};
    std::array<float, 4> driveSpeeds = {driveSpeed, driveSpeed, driveSpeed, driveSpeed};
    for (int i = 0; i < 4; i++)
    {
        modules[i].driveModule(targetAngles[i], driveSpeeds[i]);
    }
}

void SwerveDrive::setModuleStates(float targetAngles[4], float driveVelocities[4])
{
    for (int i = 0; i < 4; i++)
    {
        modules[i].directDriveModule(targetAngles[i], driveVelocities[i]);
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

SwerveModule::SwerveModule(uint8_t driveMotorPort, bool driveMotorInversion, uint8_t turnServoPort, bool turnServoInversion, float angleoffset, float wrapendpoint, float drivegearratio, float turngearratio, std::array<uint16_t, 2> servoConfig, bool brakeMode)
    : driveMotor(NoU_Motor(driveMotorPort)), turnServo(NoU_Servo(turnServoPort, servoConfig[0], servoConfig[1])) 
{
    angleOffset = angleoffset;
    wrapEndpoint = wrapendpoint;
    driveGearRatio = drivegearratio;
    turnGearRatio = turngearratio;
    turnInversion = turnServoInversion;
    driveMotor.setInverted(driveMotorInversion);
    driveMotor.setBrakeMode(brakeMode);
    driveMotor.beginEncoder();
}
void SwerveModule::driveModule(float targetAngle, float driveSpeed)
{
    bool driveInversion = false;
    float trueAngle = targetAngle - angleOffset; // make it field relative

    driveMotor.set(driveInversion ? driveSpeed : -driveSpeed); // do things and crap but field oriented!
}

void SwerveModule::directDriveModule(float targetAngle, float driveVelocity)
{
    // do things not field oriented
    float trueAngle = targetAngle - angleOffset;
    turnServo.write(trueAngle); // convert to servo angle and set
    driveMotor.set(driveVelocity);
}

float SwerveModule::getModuleAngle()
{
    return currentAngle - angleOffset;
}

float SwerveModule::getModuleVelocity()
{
    return currentSpeed;
}

void SwerveModule::setBrakeMode(bool brake)
{
    driveMotor.setBrakeMode(brake);
}

NoU_Motor *SwerveModule::getDriveMotor()
{
    return &driveMotor;
}

NoU_Servo *SwerveModule::getTurnServo()
{
    return &turnServo;
}

SwerveModule SwerveModule::getModule()
{
    return *this;
}