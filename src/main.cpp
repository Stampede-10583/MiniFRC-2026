#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <MiniSwerveLib.h>
bool robotEnabled = false;
bool firstLoop = true;
float measured_angle = 27.63;
float angular_scale = (5.0*2.0*PI) / measured_angle;
SwerveModule *swerveModule = nullptr;
NoU_Motor turnMotor(4);
//gets yaw as degrees
float getYawDeg() {
    return NoU3.yaw * angular_scale * (180.0f / PI);
}
//gets yaw as Radians
float getYawRad() {
    return NoU3.yaw * angular_scale;
}
float getTurnAngleDegrees()
{
    int32_t pos = turnMotor.getPosition();
    float angle = static_cast<float>(pos) * 800.0f;
    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }
    while (angle < 0.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

void updateEncoder() {}

void setEncoderPosition(float) { turnMotor.resetPosition(); }

bool zeroSwitchTriggered()
{
    if (digitalRead(9) == LOW)
    { // active low
        return true;
    }
    return false;
}

UniversalEncoder encoder1(
    false,
    getTurnAngleDegrees,
    updateEncoder,
    setEncoderPosition,
    zeroSwitchTriggered,
    0.0f);
std::array<float, 3> PIDconstants = {1, 1, 1};

void setup()
{
    pinMode(9, INPUT_PULLUP); // encoder zero switch pin
    Serial.begin(115200);
    NoU3.begin();
    PestoLink.begin("bomb#3");
    swerveModule = new SwerveModule(5, false, &turnMotor, false, 1.0f, 1.0f, &encoder1, false, PIDconstants);
    NoU3.setServiceLight(LIGHT_OFF);
    NoU3.calibrateIMUs(); // this takes exactly one second. Do not move the robot during calibration.
    delay(1000);          // wait for IMU calibration to finish before starting the main loop
}

void loop()
{
    swerveModule->updateModuleState();
    float driveSetpoint = 0;
    if (PestoLink.isConnected())
    {
        if (robotEnabled)
        {
            if (PestoLink.buttonHeld(13) /*&& !swerveModule->initialized*/)
            {
                PestoLink.printTerminal(String(getYawDeg()).c_str());
                // while (!swerveModule->initialized)
                // {
                //     swerveModule->initializeModule();
                // }
            }
            if (PestoLink.buttonHeld(8))
            {
                robotEnabled = false;
                PestoLink.rumble();
            }
            swerveModule->directDriveModule(PestoLink.getAxis(0), PestoLink.getAxis(2));
            if (swerveModule->initialized) swerveModule->driveMotors(); else swerveModule->stopModule();
            NoU3.setServiceLight(LIGHT_ENABLED);
        }
        else
        {
            NoU3.setServiceLight(LIGHT_DISABLED);
            swerveModule->directDriveModule(0, 0);
            swerveModule->stopModule();
            if (PestoLink.buttonHeld(9))
            {
                robotEnabled = true;
                PestoLink.rumble();
            }
        } 
       PestoLink.printTelemetry(((robotEnabled ? "E" : "D") + String(NoU3.getBatteryVoltage())).c_str(), robotEnabled ? "0x00FF00" : "0xFF0000");
    }
    else
    {
        robotEnabled = false;
        swerveModule->directDriveModule(0, 0);
        swerveModule->stopModule();
        NoU3.setServiceLight(LIGHT_DISABLED);
    }
}