#include <array>
#include <MiniSwerveLib.h>
#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include <Alfredo_NoU3_encoder.h>


SwerveDrive::SwerveDrive(
    uint8_t driveMotorPorts[4],
    std::array<bool, 4> driveMotorInversions,
    std::array<float, 4> driveGearRatios,
    uint8_t turnServoPorts[4],
    std::array<float, 4> turnServoOffsets,
    std::array<bool, 4> turnServoInversions,
    std::array<float, 4> turnServoGearRatios,
    std::array<uint16_t, 2> servoConfig,
    std::array<uint8_t, 4> turnEncoderAndZeroSwitchPorts,
    std::array<int32_t, 4> turnEncoderHomePositions,
    std::array<bool, 4> turnEncoderInversions,
    float imuAngularScale,
    float maxSpeed,
    bool brakeMode,
    uint8_t interruptPin)
    : imuAngularScale(imuAngularScale),
      maxSpeed(maxSpeed), modules{
                              SwerveModule(driveMotorPorts[0], driveMotorInversions[0], turnServoPorts[0], turnServoInversions[0], turnServoOffsets[0], driveGearRatios[0], turnServoGearRatios[0], turnEncoderHomePositions[0], servoConfig, turnEncoderAndZeroSwitchPorts[0], turnEncoderInversions[0], turnEncoderAndZeroSwitchPorts[0]),
                              SwerveModule(driveMotorPorts[1], driveMotorInversions[1], turnServoPorts[1], turnServoInversions[1], turnServoOffsets[1], driveGearRatios[1], turnServoGearRatios[1], turnEncoderHomePositions[1], servoConfig, turnEncoderAndZeroSwitchPorts[1], turnEncoderInversions[1], turnEncoderAndZeroSwitchPorts[1]),
                              SwerveModule(driveMotorPorts[2], driveMotorInversions[2], turnServoPorts[2], turnServoInversions[2], turnServoOffsets[2], driveGearRatios[2], turnServoGearRatios[2], turnEncoderHomePositions[2], servoConfig, turnEncoderAndZeroSwitchPorts[2], turnEncoderInversions[2], turnEncoderAndZeroSwitchPorts[2]),
                              SwerveModule(driveMotorPorts[3], driveMotorInversions[3], turnServoPorts[3], turnServoInversions[3], turnServoOffsets[3], driveGearRatios[3], turnServoGearRatios[3], turnEncoderHomePositions[3], servoConfig, turnEncoderAndZeroSwitchPorts[3], turnEncoderInversions[3], turnEncoderAndZeroSwitchPorts[3])}
{
    brakeMode = brakeMode;
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
    for (int i = 0; i < 4; i++)
    {
        modules[i].updateModuleState();
    }
}
SwerveDrive SwerveDrive::getDrive()
{
    return *this;
}

SwerveModule::SwerveModule(uint8_t driveMotorPort,
                           bool driveMotorInversion,
                           NoU_Motor *turnMotor,
                           bool turnMotorInversion,
                           float drivegearratio,
                           float turngearratio,
                           UniversalEncoder *turnEncoder,
                           bool brakeMode,
                           NoU_Agent *agent,
                           std::array<float, 3> kPID)
    : driveMotor(NoU_Motor(driveMotorPort)), turnMotor(turnMotor), turnEncoder(turnEncoder), agent(agent)
{
    stockEncoder = true;
    driveGearRatio = drivegearratio;
    turnGearRatio = turngearratio;

    turnInversion = turnMotorInversion;
    driveInversion = driveMotorInversion;
    driveMotor.setInverted(driveMotorInversion);
    driveMotor.setBrakeMode(brakeMode);
    posPIDController = QuickPID(&PIDInput, &turnOutput, &PIDSetpoint);
    posPIDController.SetTunings(kPID[0],kPID[1],kPID[2]);
}
SwerveModule::SwerveModule(uint8_t driveMotorPort,
                           bool driveMotorInversion,
                           uint8_t turnMotorPort,
                           bool turnMotorInversion,
                           float drivegearratio,
                           float turngearratio,
                           UniversalEncoder *turnEncoder,
                           bool brakeMode,
                           NoU_Agent *agent,
                           std::array<float, 3> kPID)
    : driveMotor(NoU_Motor(driveMotorPort)), turnEncoder(turnEncoder), agent(agent)
{
    stockEncoder = false;
    turnMotor = new NoU_Motor(turnMotorPort);
    driveGearRatio = drivegearratio;
    turnGearRatio = turngearratio;
    turnInversion = turnMotorInversion;
    driveInversion = driveMotorInversion;
    driveMotor.setInverted(driveMotorInversion);
    driveMotor.setBrakeMode(brakeMode);
    posPIDController = QuickPID(&PIDInput, &turnOutput, &PIDSetpoint);
    posPIDController.SetTunings(kPID[0],kPID[1],kPID[2]);
}
bool SwerveModule::initializeModule()
{
    if (initialized)
    {
        return true;
    }
    if (stockEncoder)
    {
        turnMotor->beginEncoder();
    }
    driveMotor.beginEncoder();
    // turnEncoder->initialize();
    turnMotor->set(.5); // optionally apply a small turn output to help the module find the zero switch if it's not already there
    while (!turnEncoder->zeroSwitch()) // wait for zero switch to be triggered
    {
        turnEncoder->update();
    }
    turnMotor->set(0);
    turnEncoder->setPosition(turnEncoder->homePosition);
    PIDInput = turnEncoder->getPosition();
    posPIDController.SetMode(QuickPID::Control::automatic);
    posPIDController.Initialize();
    initialized = true;
    return true;
}
void SwerveModule::driveModule(float targetAngle, float driveSpeed)
{
    posPIDController.SetMode(QuickPID::Control::automatic);
    bool instantialDriveInversion = false;
    // determine the inversion here
    //  also add the position PID for the motor or some crap
    PIDSetpoint = targetAngle;
    float instantialDriveSpeed = instantialDriveInversion ? driveSpeed : -driveSpeed;
    driveSetpoint = driveInversion ? instantialDriveSpeed : -instantialDriveSpeed; // do things and crap but field oriented!
}

void SwerveModule::directDriveModule(float turnVelocity, float driveVelocity)
{
    posPIDController.SetMode(QuickPID::Control::manual);
    turnOutput = turnVelocity;
    driveSetpoint = driveInversion ? driveVelocity : -driveVelocity;
}

float SwerveModule::getModuleAngle()
{
    this->updateModuleState();
    return currentAngle;
}

float SwerveModule::getModuleVelocity()
{
    return currentSpeed;
}

void SwerveModule::setBrakeMode(bool brake)
{
    driveMotor.setBrakeMode(brake);
}
void SwerveModule::updateModuleState()
{
    turnEncoder->update();
    if (!(turnEncoder->getPosition() <= 360 && turnEncoder->getPosition() >= 0))
    {
        Serial.println("Encoder angle invalid");
        while (true)
        {
            delay(1);
        };
    }
    currentAngle = turnEncoder->getPosition();
    // do this in order to get velo currentSpeed = driveMotor.getPosition() / driveGearRatio;     // convert motor encoder velocity to wheel speed
}
void SwerveModule::driveMotors() {
    posPIDController.Compute();
    posPIDController.GetMode() ?  turnMotor->set(turnOutput/255) : turnMotor->set(turnOutput); // if in automatic mode, turnOutput is a value from 0-255 representing the power to apply to the motor. if in manual mode, turnOutput is the actual velocity to set on the motor. this allows for more direct control when not using the PID controller, which can be useful for testing and debugging.
    driveMotor.set(driveSetpoint);
}
NoU_Motor *SwerveModule::getDriveMotor()
{
    return &driveMotor;
}

NoU_Motor *SwerveModule::getTurnMotor()
{
    return turnMotor;
}
UniversalEncoder *SwerveModule::getTurnEncoder()
{
    return turnEncoder;
}
SwerveModule SwerveModule::getModule()
{
    return *this;
}
QuickPID *SwerveModule::getPIDController() {
    return &posPIDController;
}
UniversalEncoder::UniversalEncoder(bool isStock, std::function<float()> getPositionFunc, std::function<void()> updateFunc, std::function<void(float)> setPositionFunc, std::function<bool()> zeroSwitchSupplier, float homePosition)
    : getPosition(getPositionFunc), setPosition(setPositionFunc), update(updateFunc), zeroSwitch(zeroSwitchSupplier), homePosition(homePosition)
{
    isStockEncoder = isStock;
    isAbsolute = false;
}
UniversalEncoder::UniversalEncoder(std::function<float()> getPositionFunc, std::function<void()> updateFunc)
    : getPosition(getPositionFunc), update(updateFunc)
{
    isAbsolute = true;
}