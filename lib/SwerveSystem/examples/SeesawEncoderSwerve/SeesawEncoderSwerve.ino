#include <Arduino.h>
#include <Adafruit_seesaw.h>
#include "MiniSwerveLib.h"

// Replace these port numbers with the actual NoU ports used on your robot.
static uint8_t driveMotorPorts[4] = {1, 2, 3, 4};
static uint8_t turnServoPorts[4] = {5, 6, 7, 8};

static std::array<bool, 4> driveMotorInversions = {false, false, false, false};
static std::array<float, 4> driveGearRatios = {1.0f, 1.0f, 1.0f, 1.0f};
static std::array<float, 4> turnServoOffsets = {0.0f, 0.0f, 0.0f, 0.0f};
static std::array<bool, 4> turnServoInversions = {false, false, false, false};
static std::array<float, 4> turnServoGearRatios = {1.0f, 1.0f, 1.0f, 1.0f};
static std::array<uint16_t, 2> servoConfig = {540, 2300};
static std::array<bool, 4> turnEncoderInversions = {false, false, false, false};

Adafruit_seesaw encoderSeesaw;
SwerveDrive *swerveDrive = nullptr;

void setup()
{
    Serial.begin(115200);

    if (!encoderSeesaw.begin(SEESAW_ADDRESS))
    {
        Serial.println("Failed to initialize seesaw encoder board");
        while (true)
        {
        }
    }

    // The library assumes the seesaw INT line is wired to ESP GPIO 9.
    // The encoder channel pairs are fixed in the library as:
    // 0: A=8,  B=9
    // 1: A=10, B=11
    // 2: A=2,  B=3
    // 3: A=4,  B=5
    swerveDrive = new SwerveDrive(
        driveMotorPorts,
        driveMotorInversions,
        driveGearRatios,
        turnServoPorts,
        turnServoOffsets,
        turnServoInversions,
        turnServoGearRatios,
        servoConfig,
        turnEncoderInversions,
        &encoderSeesaw,
        1.0f,
        1.0f);

    Serial.println("SwerveDrive initialized with seesaw-backed encoders");
}

void loop()
{
    if (swerveDrive != nullptr)
    {
        swerveDrive->swerveDrivePeriodic();
    }

    delay(20);
}
