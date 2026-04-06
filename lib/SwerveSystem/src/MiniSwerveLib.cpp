#include <array>
#include "MiniSwerveLib.h"
#include "Arduino.h"
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
#include "Adafruit_seesaw.h"

namespace
{
    constexpr uint8_t kMaxTrackedEncoders = 8;
    QuicEncoder *gTrackedEncoders[kMaxTrackedEncoders] = {nullptr};
    uint8_t gTrackedEncoderCount = 0;
    constexpr std::array<std::array<uint8_t, 3>, 4> kSeesawEncoderAndZeroSwitchPins = {{{8, 9, 12}, {10, 11, 14}, {2, 3, 17}, {4, 5, 9}}};
    Adafruit_seesaw *gSeesaw = nullptr;
    int8_t gSeesawInterruptPin = -1;
    bool gSeesawInterruptAttached = false;

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
        Adafruit_seesaw *Seesaw,
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
        configureSeesaw(Seesaw, interruptPin);
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
    void SwerveDrive::configureSeesaw(Adafruit_seesaw *seesaw, int8_t interruptPin)
    {
        gSeesaw = seesaw;
        SwerveDrive::configureSeesawInterruptPin(interruptPin);
    }
    void SwerveDrive::configureSeesawInterruptPin(int8_t interruptPin)
    {
        gSeesawInterruptPin = interruptPin;
        gSeesawInterruptAttached = false;
    }
    SwerveModule::SwerveModule(uint8_t driveMotorPort,
                               bool driveMotorInversion,
                               uint8_t turnServoPort,
                               bool turnServoInversion,
                               float angleoffset,
                               float drivegearratio,
                               float turngearratio,
                               float encodergearratio,
                               std::array<uint16_t, 2> servoConfig,
                               uint8_t turnEncoderAndZeroSwitchPort,
                               int32_t turnEncoderHomePosition,
                               bool turnEncoderInversion,
                               bool brakeMode)
        : driveMotor(NoU_Motor(driveMotorPort)), turnServo(NoU_Servo(turnServoPort, servoConfig[0], servoConfig[1])), turnEncoder(QuicEncoder(kSeesawEncoderAndZeroSwitchPins[turnEncoderAndZeroSwitchPort][0], kSeesawEncoderAndZeroSwitchPins[turnEncoderAndZeroSwitchPort][1], turnEncoderInversion))
    {
        angleOffset = angleoffset;
        driveGearRatio = drivegearratio;
        turnGearRatio = turngearratio;
        encoderGearRatio = encodergearratio;
        turnInversion = turnServoInversion;
        encoderPinA = kSeesawEncoderAndZeroSwitchPins[turnEncoderAndZeroSwitchPort][0];
        encoderPinB = kSeesawEncoderAndZeroSwitchPins[turnEncoderAndZeroSwitchPort][1];
        zeroSwitchPin = kSeesawEncoderAndZeroSwitchPins[turnEncoderAndZeroSwitchPort][2];
        encoderHomePosition = turnEncoderHomePosition;

        driveMotor.setInverted(driveMotorInversion);
        driveMotor.setBrakeMode(brakeMode);
    }
    void SwerveModule::initializeModule()
    {
        gSeesaw->pinMode(zeroSwitchPin, INPUT_PULLUP);
        driveMotor.beginEncoder();
        turnEncoder.initialize();
        while (!gSeesaw->digitalRead(zeroSwitchPin)) // wait for zero switch to be triggered
        {
            turnServo.write(-15); // optionally apply a small turn output to help the module find the zero switch if it's not already there
        }
        turnEncoder.resetPosition(encoderHomePosition);
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
    void SwerveModule::updateModuleState()
    {
        turnEncoder.update();
        currentAngle = turnEncoder.getPosition() % (int32_t)(encoderGearRatio * 360); // wrap encoder position to 0-360 based on gear ratio
        // do this in order to get velo currentSpeed = driveMotor.getPosition() / driveGearRatio;     // convert motor encoder velocity to wheel speed
    }
    NoU_Motor *SwerveModule::getDriveMotor()
    {
        return &driveMotor;
    }

    NoU_Servo *SwerveModule::getTurnServo()
    {
        return &turnServo;
    }
    QuicEncoder *SwerveModule::getTurnEncoder()
    {
        return &turnEncoder;
    }
    SwerveModule SwerveModule::getModule()
    {
        return *this;
    }
    QuicEncoder::QuicEncoder(uint8_t pinA, uint8_t pinB, bool inverted)
        : pinA(pinA), pinB(pinB), inverted(inverted), position(0), prevState(0)
    {
        if (gSeesaw == nullptr)
        {
            pinMode(pinA, INPUT_PULLUP);
            pinMode(pinB, INPUT_PULLUP);
        }

        prevState = (readEncoderPin(pinA) << 1) | readEncoderPin(pinB);
        attachEncoderInterrupt(pinA, this);
        attachEncoderInterrupt(pinB, this);
    }
    void QuicEncoder::initialize()
    {
    }

    int32_t QuicEncoder::getPosition()
    {
        noInterrupts();
        int32_t pos = position;
        interrupts();
        return pos;
    }
    void QuicEncoder::resetPosition(int32_t newPosition)
    {
        noInterrupts();
        position = newPosition;
        interrupts();
    }
    void QuicEncoder::update()
    {
        uint8_t state = (readEncoderPin(pinA) << 1) | readEncoderPin(pinB);
        uint8_t transition = (prevState << 2) | state;

        static const int8_t dirLookup[16] = {
            0, -1, 1, 0,
            1, 0, 0, -1,
            -1, 0, 0, 1,
            0, 1, -1, 0};

        int8_t directionStep = dirLookup[transition];
        if (inverted)
        {
            directionStep = -directionStep;
        }

        position += directionStep;
        prevState = state;
    }

    void quicEncoderISRDispatch()
    {
        for (uint8_t i = 0; i < gTrackedEncoderCount; ++i)
        {
            if (gTrackedEncoders[i] != nullptr)
            {
                gTrackedEncoders[i]->update();
            }
        }
    }

    void trackEncoderIfNeeded(QuicEncoder *encoder)
    {
        for (uint8_t i = 0; i < gTrackedEncoderCount; ++i)
        {
            if (gTrackedEncoders[i] == encoder)
            {
                return;
            }
        }

        if (gTrackedEncoderCount < kMaxTrackedEncoders)
        {
            gTrackedEncoders[gTrackedEncoderCount++] = encoder;
        }
    }

    void attachEncoderInterrupt(uint8_t pin, QuicEncoder *encoder)
    {
        trackEncoderIfNeeded(encoder);

        if (gSeesaw != nullptr)
        {
            if (pin < 32)
            {
                gSeesaw->setGPIOInterrupts((1UL << pin), true);
            }

            if (!gSeesawInterruptAttached && gSeesawInterruptPin >= 0)
            {
                int seesawInterruptNumber = digitalPinToInterrupt(gSeesawInterruptPin);
                if (seesawInterruptNumber >= 0)
                {
                    attachInterrupt(seesawInterruptNumber, quicEncoderISRDispatch, FALLING);
                    gSeesawInterruptAttached = true;
                }
            }
            return;
        }

        int interruptNumber = digitalPinToInterrupt(pin);
        if (interruptNumber < 0)
        {
            return;
        }

        attachInterrupt(interruptNumber, quicEncoderISRDispatch, CHANGE);
    }

    uint8_t readEncoderPin(uint8_t pin)
    {
        if (gSeesaw != nullptr)
        {
            return gSeesaw->digitalRead(pin) ? HIGH : LOW;
        }

        return digitalRead(pin);
    }
}