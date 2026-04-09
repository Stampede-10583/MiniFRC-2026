#include <MiniSwerveLib.h>

SwerveDrive::SwerveDrive(
    NoU_Agent* agent,
    float imuAngularScale,
    uint8_t driveMotorPorts[4],
    std::array<bool, 4> driveMotorInversions,
    std::array<float, 4> driveGearRatios,
    std::array<NoU_Motor *, 4> turnMotors,
    std::array<bool, 4> turnMotorInversions,
    std::array<float, 4> turnMotorGearRatios,
    std::array<UniversalEncoder*, 4> turnEncoders,
    float maxSpeed,
    bool brakeMode,
    std::array<std::array<float, 2>, 4> moduleOffsets,
    std::array<float, 3> kPID)
    : agent(agent), imuAngularScale(imuAngularScale),
      maxSpeed(maxSpeed), brakeMode(brakeMode), moduleOffsets(moduleOffsets), modules{
                              SwerveModule(driveMotorPorts[0], driveMotorInversions[0], turnMotors[0], turnMotorInversions[0], driveGearRatios[0], turnMotorGearRatios[0], turnEncoders[0], brakeMode, kPID),
                              SwerveModule(driveMotorPorts[1], driveMotorInversions[1], turnMotors[1], turnMotorInversions[1], driveGearRatios[1], turnMotorGearRatios[1], turnEncoders[1], brakeMode, kPID),
                              SwerveModule(driveMotorPorts[2], driveMotorInversions[2], turnMotors[2], turnMotorInversions[2], driveGearRatios[2], turnMotorGearRatios[2], turnEncoders[2], brakeMode, kPID),
                              SwerveModule(driveMotorPorts[3], driveMotorInversions[3], turnMotors[3], turnMotorInversions[3], driveGearRatios[3], turnMotorGearRatios[3], turnEncoders[3], brakeMode, kPID)}
{
}
SwerveDrive::SwerveDrive(
    NoU_Agent* agent,
    float imuAngularScale,
    uint8_t driveMotorPorts[4],
    std::array<bool, 4> driveMotorInversions,
    std::array<float, 4> driveGearRatios,
    uint8_t turnMotorPorts[4],
    std::array<bool, 4> turnMotorInversions,
    std::array<float, 4> turnMotorGearRatios,
    std::array<UniversalEncoder*, 4> turnEncoders,
    float maxSpeed,
    bool brakeMode,
    std::array<std::array<float, 2>, 4> moduleOffsets,
    std::array<float, 3> kPID)
    : agent(agent), imuAngularScale(imuAngularScale),
      maxSpeed(maxSpeed), brakeMode(brakeMode), moduleOffsets(moduleOffsets), modules{
                              SwerveModule(driveMotorPorts[0], driveMotorInversions[0], turnMotorPorts[0], turnMotorInversions[0], driveGearRatios[0], turnMotorGearRatios[0], turnEncoders[0], brakeMode, kPID),
                              SwerveModule(driveMotorPorts[1], driveMotorInversions[1], turnMotorPorts[1], turnMotorInversions[1], driveGearRatios[1], turnMotorGearRatios[1], turnEncoders[1], brakeMode, kPID),
                              SwerveModule(driveMotorPorts[2], driveMotorInversions[2], turnMotorPorts[2], turnMotorInversions[2], driveGearRatios[2], turnMotorGearRatios[2], turnEncoders[2], brakeMode, kPID),
                              SwerveModule(driveMotorPorts[3], driveMotorInversions[3], turnMotorPorts[3], turnMotorInversions[3], driveGearRatios[3], turnMotorGearRatios[3], turnEncoders[3], brakeMode, kPID)}
{
}
std::vector<float> SwerveDrive::getTargetVelocities(float xJoystick, float yJoystick, float rotationJoystick, float scale, bool robotOriented)
{
    std::vector<float> velocities(3, 0);
    float x = xJoystick * scale;
    float y = yJoystick * scale;
    float rotation = rotationJoystick * scale;
    if (!robotOriented)
    {
        float robotAngleRad = heading * PI / 180.0f;
        float tempX = x * cos(robotAngleRad) - y * sin(robotAngleRad);
        float tempY = x * sin(robotAngleRad) + y * cos(robotAngleRad);
        x = tempX;
        y = tempY;
    }
    velocities[0] = x;
    velocities[1] = y;
    velocities[2] = rotation;
    return velocities;
}
void SwerveDrive::drive(std::vector<float> velocities, float driveSpeed)
{
    std::array<float, 4> targetAngles;
    std::array<float, 4> driveSpeeds;
    for (int i = 0; i < 4; i++)
    {
        Vec2f moduleOffset(moduleOffsets[i][0], moduleOffsets[i][1]);
        Vec2f robotVelocity(velocities[0], velocities[1]);
        Vec2f OutputVelocity = robotVelocity + velocities[2] * moduleOffset.getPerpendicular();
        targetAngles[i] = atan2(OutputVelocity.y, OutputVelocity.x) * 180.0f / PI;
        driveSpeeds[i] = OutputVelocity.distance(Vec2f(0, 0)) * driveSpeed;
    }
    float maxWheelSpeed = *std::max_element(driveSpeeds.begin(), driveSpeeds.end());
    if (maxWheelSpeed > maxSpeed)
    {
        for (int i = 0; i < 4; i++)
        {
            driveSpeeds[i] /= maxWheelSpeed / maxSpeed;
        }
    }
    for (int i = 0; i < 4; i++)
    {
        modules[i].driveModule(targetAngles[i], driveSpeeds[i]);
    }
}
void SwerveDrive::driveModules(float targetAngles[4], float driveSpeeds[4])
{
    for (int i = 0; i < 4; i++)
    {
        modules[i].driveModule(targetAngles[i], driveSpeeds[i]);
    }
}
void SwerveDrive::directDriveModules(float turnSpeeds[4], float driveSpeeds[4])
{
    for (int i = 0; i < 4; i++)
    {
        modules[i].directDriveModule(turnSpeeds[i], driveSpeeds[i]);
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
}

void SwerveDrive::setMaxSpeed(float maxSpeed)
{
    this->maxSpeed = maxSpeed;
}

std::array<float, 3> SwerveDrive::getOdeometry()
{
    return {xPos, yPos, heading};
}

void SwerveDrive::resetOdeometry(float x, float y, float angle)
{

    // TODO: Reset odometry state.
}

void SwerveDrive::swerveDrivePeriodic(bool robotEnabled)
{
    for (int i = 0; i < 4; i++)
    {
        modules[i].updateModuleState();
        if (robotEnabled)
        {
            modules[i].driveMotors();
        }
        else
        {
            modules[i].stopModule();
        }
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
                           std::array<float, 3> kPID)
    : driveMotor(NoU_Motor(driveMotorPort)), turnMotor(turnMotor), turnEncoder(turnEncoder)
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
                           std::array<float, 3> kPID)
    : driveMotor(NoU_Motor(driveMotorPort)), turnEncoder(turnEncoder)
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