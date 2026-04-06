#include <array>
#include "SingleModuleTest.h"
#include "Arduino.h"
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
//#include "Adafruit_seesaw.h"
//constexpr std::array<std::array<uint8_t, 3>, 4> kSeesawEncoderAndZeroSwitchPins = {{{8, 9, 12}, {10, 11, 14}, {2, 3, 17}, {4, 5, 9}}};


SwerveModule::SwerveModule(uint8_t driveMotorPort,
                 bool driveMotorInversion = false,
                 uint8_t turnMotorPort,
                 bool turnMotorInversion = false,
                 float angleoffset,
                 float drivegearratio,
                 float turngearratio,
                 float encodergearratio,
                 UniversalEncoder* turnEncoder,
                 float turnEncoderHomePosition,
                 bool brakeMode = false,
                 bool driveinversion)
    : driveMotor(NoU_Motor(driveMotorPort)), turnMotor(NoU_Motor(turnMotorPort)), turnEncoder(turnEncoder)
{
    driveInversion = driveinversion;
    driveGearRatio = drivegearratio;
    turnGearRatio = turngearratio;

    turnInversion = turnMotorInversion;
    encoderHomePosition = turnEncoderHomePosition;

    driveMotor.setInverted(driveMotorInversion);
    driveMotor.setBrakeMode(brakeMode);
}

namespace
{
    void SwerveModule::initializeModule()
    {
        //gSeesaw->pinMode(zeroSwitchPin, INPUT_PULLUP);
        driveMotor.beginEncoder();
        //turnEncoder->initialize();
        while (!turnEncoder->zeroSwitch()) // wait for zero switch to be triggered
        {
            turnMotor.set(.1); // optionally apply a small turn output to help the module find the zero switch if it's not already there
        }
        turnEncoder->setPosition(encoderHomePosition);
    }
    void SwerveModule::driveModule(float targetAngle, float driveSpeed)
    {
        bool instantialDriveInversion = false;
        float instantialDriveSpeed = instantialDriveInversion? driveSpeed : -driveSpeed;
        driveMotor.set(driveInversion ? instantialDriveSpeed : -instantialDriveSpeed); // do things and crap but field oriented!
    }

    void SwerveModule::directDriveModule(float targetAngle, float driveVelocity)
    {
        driveMotor.set(driveInversion ? driveVelocity : -driveVelocity);
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
        currentAngle = (float)((int32_t)turnEncoder->getPosition() % (int32_t)(encoderGearRatio * 360)); // wrap encoder position to 0-360 based on gear ratio
        // do this in order to get velo currentSpeed = driveMotor.getPosition() / driveGearRatio;     // convert motor encoder velocity to wheel speed
    }
    NoU_Motor *SwerveModule::getDriveMotor()
    {
        return &driveMotor;
    }

    NoU_Motor *SwerveModule::getTurnMotor()
    {
        return &turnMotor;
    }
    // QuicEncoder *SwerveModule::getTurnEncoder()
    // {
    //     return &turnEncoder;
    // }
    SwerveModule SwerveModule::getModule()
    {
        return *this;
    }
    UniversalEncoder::UniversalEncoder(std::function<float()> getPositionFunc, std::function<void(float)> setPositionFunc, std::function<void()> updateFunc, std::function<bool()> zeroSwitchSupplier)
        : getPosition(getPositionFunc), setPosition(setPositionFunc), update(updateFunc), zeroSwitch(zeroSwitchSupplier)
    {
    }
    // QuicEncoder::QuicEncoder(uint8_t pinA, uint8_t pinB, bool inverted)
    //     : pinA(pinA), pinB(pinB), inverted(inverted), position(0), prevState(0)
    // {
    //     if (gSeesaw == nullptr)
    //     {
    //         pinMode(pinA, INPUT_PULLUP);
    //         pinMode(pinB, INPUT_PULLUP);
    //     }

    //     prevState = (readEncoderPin(pinA) << 1) | readEncoderPin(pinB);
    //     attachEncoderInterrupt(pinA, this);
    //     attachEncoderInterrupt(pinB, this);
    // }
    // void QuicEncoder::initialize()
    // {
    // }

    // int32_t QuicEncoder::getPosition()
    // {
    //     noInterrupts();
    //     int32_t pos = position;
    //     interrupts();
    //     return pos;
    // }
    // void QuicEncoder::resetPosition(int32_t newPosition)
    // {
    //     noInterrupts();
    //     position = newPosition;
    //     interrupts();
    // }
    // void QuicEncoder::update()
    // {
    //     uint8_t state = (readEncoderPin(pinA) << 1) | readEncoderPin(pinB);
    //     uint8_t transition = (prevState << 2) | state;

    //     static const int8_t dirLookup[16] = {
    //         0, -1, 1, 0,
    //         1, 0, 0, -1,
    //         -1, 0, 0, 1,
    //         0, 1, -1, 0};

    //     int8_t directionStep = dirLookup[transition];
    //     if (inverted)
    //     {
    //         directionStep = -directionStep;
    //     }

    //     position += directionStep;
    //     prevState = state;
    // }

    // void quicEncoderISRDispatch()
    // {
    //     for (uint8_t i = 0; i < gTrackedEncoderCount; ++i)
    //     {
    //         if (gTrackedEncoders[i] != nullptr)
    //         {
    //             gTrackedEncoders[i]->update();
    //         }
    //     }
    // }

    // void trackEncoderIfNeeded(QuicEncoder *encoder)
    // {
    //     for (uint8_t i = 0; i < gTrackedEncoderCount; ++i)
    //     {
    //         if (gTrackedEncoders[i] == encoder)
    //         {
    //             return;
    //         }
    //     }

    //     if (gTrackedEncoderCount < kMaxTrackedEncoders)
    //     {
    //         gTrackedEncoders[gTrackedEncoderCount++] = encoder;
    //     }
    // }

    // void attachEncoderInterrupt(uint8_t pin, QuicEncoder *encoder)
    // {
    //     trackEncoderIfNeeded(encoder);

    //     if (gSeesaw != nullptr)
    //     {
    //         if (pin < 32)
    //         {
    //             gSeesaw->setGPIOInterrupts((1UL << pin), true);
    //         }

    //         if (!gSeesawInterruptAttached && gSeesawInterruptPin >= 0)
    //         {
    //             int seesawInterruptNumber = digitalPinToInterrupt(gSeesawInterruptPin);
    //             if (seesawInterruptNumber >= 0)
    //             {
    //                 attachInterrupt(seesawInterruptNumber, quicEncoderISRDispatch, FALLING);
    //                 gSeesawInterruptAttached = true;
    //             }
    //         }
    //         return;
    //     }

    //     int interruptNumber = digitalPinToInterrupt(pin);
    //     if (interruptNumber < 0)
    //     {
    //         return;
    //     }

    //     attachInterrupt(interruptNumber, quicEncoderISRDispatch, CHANGE);
    // }

    // uint8_t readEncoderPin(uint8_t pin)
    // {
    //     if (gSeesaw != nullptr)
    //     {
    //         return gSeesaw->digitalRead(pin) ? HIGH : LOW;
    //     }

    //     return digitalRead(pin);
    // }
}