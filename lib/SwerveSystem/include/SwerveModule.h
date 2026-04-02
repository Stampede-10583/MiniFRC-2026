#ifndef  Morse_h
#define Morse_h
#include "Arduino.h"
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
class SwerveModule {
    public:
        SwerveModule(NoU_Motor* drive, NoU_Servo* turn, Encoder* driveencoder, float angleoffset, float wrapendpoint,  float drivegearratio, float turngearratio);
        void driveModule(float targetAngle, float driveSpeed);
        void directDriveModule(float targetAngle, float driveSpeed);
        float getModuleAngle();
        float getModuleVelocity();
        SwerveModule getModule();
    private: 
        NoU_Motor *driveMotor;
        NoU_Servo *turnMotor;
        Encoder *driveEncoder;
        float angleOffset;
        float wrapEndpoint;
        float driveGearRatio;
        float turnGearRatio;
        float currentSpeed;
        float currentAngle;
};
#endif