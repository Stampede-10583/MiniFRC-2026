#include "Arduino.h"
#include "SwerveModule.h"
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
SwerveModule::SwerveModule(uint8_t driveMotorPort, bool driveMotorInversion, uint8_t turnServoPort, bool turnServoInversion, float angleoffset, float wrapendpoint, float drivegearratio, float turngearratio, bool brakeMode)
    : driveMotor(NoU_Motor(driveMotorPort)), turnServo(NoU_Servo(turnServoPort))
{
    angleOffset = angleoffset;
    wrapEndpoint = wrapendpoint;
    driveGearRatio = drivegearratio;
    turnGearRatio = turngearratio;
    turnInversion = turnServoInversion;
    driveMotor.setInverted(driveMotorInversion);
    driveMotor.setBrakeMode(brakeMode);
    driveMotor.beginEncoder();
};
void SwerveModule::driveModule(float targetAngle, float driveSpeed)
{
    bool driveInversion = false;
    float trueAngle = targetAngle - angleOffset; // make it field relative

    driveMotor.set(driveInversion ? driveSpeed : -driveSpeed); // do things and crap but field oriented!
};

void SwerveModule::directDriveModule(float targetAngle, float driveVelocity)
{
    // do things not field oriented
    float trueAngle = targetAngle - angleOffset;
    driveMotor.set(driveVelocity);
};

float SwerveModule::getModuleAngle()
{
    return currentAngle - angleOffset;
};

float SwerveModule::getModuleVelocity()
{
    return currentSpeed;
};

void SwerveModule::setBrakeMode(bool brake)
{
    driveMotor.setBrakeMode(brake);
};

NoU_Motor *SwerveModule::getDriveMotor()
{
    return &driveMotor;
};

NoU_Servo *SwerveModule::getTurnServo()
{
    return &turnServo;
};

SwerveModule SwerveModule::getModule()
{
    return *this;
};