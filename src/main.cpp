#include <SingleModuleTest.h>
#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
SwerveModule *swerveModule = nullptr;
NoU_Motor turnMotor(3);

float getTurnAngleDegrees() {
    int32_t pos = turnMotor.getPosition();
    float angle = static_cast<float>(pos) * 882.0f;

    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    while (angle < 0.0f) {
        angle += 360.0f;
    }

    return angle;
}

void updateEncoder() {}

void setEncoderPosition(float) {}

bool zeroSwitchTriggered() {
    return false;
}

UniversalEncoder encoder1(
    false,
    getTurnAngleDegrees,
    updateEncoder,
    setEncoderPosition,
    zeroSwitchTriggered,
    0.0f);

void setup() {
    Serial.begin(115200);
    NoU3.begin();
    PestoLink.begin("bomba");
    turnMotor.beginEncoder();
    swerveModule = new SwerveModule(4, false, &turnMotor, false, 1.0f, 1.0f, &encoder1, false);
}
void loop() {
    swerveModule->updateModuleState();
}