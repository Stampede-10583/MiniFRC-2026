#include "Arduino.h"
#include "SwerveModule.h"
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
/*Constructor: takes in a NoU_Motor for the drive, a NoU_Servo for the turn, and Encoder for the drive encoder.
    @param angleoffset the angle of the servo at zero position in degrees
    @param wrapendpoint the angle where the servo cant move anymore in degrees
    @param drive/turngearratio gear ratio of motor to mechanism
*/
SwerveModule::SwerveModule(NoU_Motor *drive, NoU_Servo *turn, Encoder *driveencoder, float angleoffset, float wrapendpoint, float drivegearratio, float turngearratio)
    : driveMotor(drive), turnMotor(turn), driveEncoder(driveencoder)
{
    angleOffset = angleoffset;
    wrapEndpoint = wrapendpoint;
    driveGearRatio = drivegearratio;
    turnGearRatio = turngearratio;
    driveMotor->beginEncoder();
};
/* Move the module field oriented. The speed is scalar, and the angle will be adjusted internally to account for the offset and deadspot etc
    @param targetAngle the desired field-oriented angle in degrees
    @param driveSpeed the desired NON DIRECTIONAL speed in m/s
*/
void SwerveModule::driveModule(float targetAngle, float driveSpeed) {
    bool driveInversion =false;
    float trueAngle = targetAngle - angleOffset; // make it field relative
    
    driveMotor->set(driveInversion? driveSpeed : -driveSpeed);// do things and crap but field oriented!
};

/* Move the module explicitly. The angle will be adjusted internally to account for the offset
    @param targetAngle the desired ROBOT ORIENTED angle in degrees
    @param driveVelocity the desired DIRECTIONAL VELOCITY in m/s
*/
void SwerveModule::directDriveModule(float targetAngle, float driveVelocity) {
    // do things not field oriented
    float trueAngle = targetAngle - angleOffset;
    driveMotor->set(driveVelocity);
};

/* Returns the current angle robot oriented of the module in degrees.
 */
float SwerveModule::getModuleAngle()
{
    return currentAngle - angleOffset;
};
/* Returns the current running velocity of the drive motor in m/s
 */
float SwerveModule::getModuleVelocity()
{
    return currentSpeed;
};
/*Returns a pointer to the module
 */
SwerveModule SwerveModule::getModule()
{
    return *this;
};