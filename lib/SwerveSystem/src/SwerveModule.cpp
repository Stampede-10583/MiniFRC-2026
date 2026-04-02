#include "Arduino.h"
#include "SwerveModule.h"
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"

SwerveModule::SwerveModule(NoU_Motor *drive, NoU_Servo *turn, Encoder *driveencoder, float angleoffset, float wrapendpoint, float gearratio)
 : driveMotor(drive), turnMotor(turn), driveEncoder(driveencoder)
{
    angleOffset = angleoffset;
    wrapEndpoint = wrapendpoint;
    gearRatio = gearratio;
};
void SwerveModule::driveModule(float targetAngle, float driveSpeed) {

};
void SwerveModule::directDriveModule(float targetAngle, float driveSpeed) {

};
float SwerveModule::getModuleAngle() {

};
float SwerveModule::getModuleSpeed() {

};
SwerveModule SwerveModule::getModule() {

};