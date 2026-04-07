#include "SingleModuleTest.h"
#include "Arduino.h"
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"

SwerveModule *swerve = nullptr;
void setup() {
    Serial.begin(115200);
    NoU3.begin();
    PestoLink.begin("Name me!");
    NoU_Motor turnMotor = NoU_Motor(3);
    UniversalEncoder Encoder1 = UniversalEncoder(false, [](){return 1.0;}, [](){}, [](float newPosition){}, [](){return false;}, 0.0f);
    swerve = new SwerveModule(4, false, &turnMotor, false, 1.0f, 1.0f, &Encoder1, false);
}
void loop() {
    swerve->updateModuleState();
}