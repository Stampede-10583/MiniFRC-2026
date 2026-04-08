#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <SingleModuleTest.h>

// This creates the drivetrain object, you shouldn't have to mess with this

//The gyroscope sensor is by default precise, but not accurate. This is fixable by adjusting the angular scale factor.
//Tuning procedure: 
//Rotate the robot in place exactly 5 times. Use the Serial printout to read the current gyro angle in Radians, we will call this "measured_angle".
//measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process. 
// float measured_angle = 31.416;
// float angular_scale = (5.0*2.0*PI) / measured_angle;
SwerveModule *swerveModule = nullptr;
NoU_Motor turnMotor(3);
bool telemetryEnabled = true;
float getTurnAngleDegrees() {
    int32_t pos = turnMotor.getPosition();
    float angle = static_cast<float>(pos) * 800.0f;
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle;
}

void updateEncoder() {}

void setEncoderPosition(float) {turnMotor.resetPosition();}

bool zeroSwitchTriggered() {
    if (digitalRead(9) == LOW) { // active low
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

void setup() {
    pinMode(9, INPUT); // encoder zero switch pin
    Serial.begin(115200);
    NoU3.begin();
    PestoLink.begin("bomb#3");
    swerveModule = new SwerveModule(5, false, &turnMotor, false, 1.0f, 1.0f, &encoder1, false);
    NoU3.setServiceLight(LIGHT_CALIBRATING);
    NoU3.calibrateIMUs(); // this takes exactly one second. Do not move the robot during calibration.
    delay(1000); // wait for IMU calibration to finish before starting the main loop
}

void loop() {
    swerveModule->updateModuleState();
    float driveSetpoint = 0;
    if (PestoLink.isConnected()) {
        PestoLink.printBatteryVoltage(NoU3.getBatteryVoltage());
        NoU3.setServiceLight(LIGHT_ENABLED);
        driveSetpoint = PestoLink.getAxis(0); // drive with left stick
        if (PestoLink.buttonHeld(13)) { //POV DOWN for telemetry
            telemetryEnabled = !telemetryEnabled;
            PestoLink.rumble();
        }
        if (telemetryEnabled) {
            String telemetryText = "Yaw: " + String(NoU3.yaw) + " Pitch: " + String(NoU3.pitch) + " Roll: " + String(NoU3.roll);
            PestoLink.printTerminal(telemetryText.c_str());
        }
        if (PestoLink.buttonHeld(1)) {
            swerveModule->initializeModule();
        }
    } else {
         // stop the robot if the controller is disconnected
        NoU3.setServiceLight(LIGHT_DISABLED);
    }
    swerveModule->driveModule(driveSetpoint, 0);
}