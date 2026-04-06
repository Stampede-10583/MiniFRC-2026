#ifndef SingleModuleTest_h
#define SingleModuleTest_h
#include "Arduino.h"
#include <array>
#include <vector>
#include <functional>
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
class Adafruit_seesaw;
class SwerveModule
{
public:
    /**
     * Construct a swerve module controller.
     *
     * @param driveMotorPort Port for the drive motor controller.
     * @param driveMotorInversion True to invert the drive motor direction.
     * @param turnMotorPort Port for the steering motor.
     * @param turnMotorInversion True to invert steering direction.
     * @param angleoffset Steering zero offset in degrees.
     * @param drivegearratio Drive gear ratio from motor to wheel.
     * @param turngearratio Steering gear ratio from motor to module.
     * @param encodergearratio Encoder gear ratio for position feedback.
     * @param motorConfig Array containing {motorMinPulse (corresponds to 0°), motorMaxPulse (corresponds to 180°)} PWM bounds for steering motors. Defaults to {540, 2300} which is a common range 9g motor.
     * @param turnEncoder UniversalEncoder instance for the steering encoder. This is used to allow flexible configuration of the encoder implementation while keeping the module code generic.
     *                           The getPosition function should return the current encoder angle in degrees, accounting for gear ratios, offsets, and inversions so that it can be directly compared to the target angles in the driveModule function. The setPosition function should accept an angle in degrees and set the encoder position accordingly, and is used to zero the encoder during initialization.
     * @param turnEncoderAndZeroSwitchSupplier Boolean supplier function that returns true when the turn encoder zero switch is triggered. This is used during initialization to find the zero position of the module.
     * @param turnEncoderHomePosition The encoder position corresponding to the home position in degrees. This is used to zero the encoder during initialization.
     * @param turnEncoderInversions Per-module turn encoder inversion flags. Set so that it is CCW+ for all modules.
     * @param brakeMode True to enable drive motor braking.
     * @param driveInversion Set this so that the drive wheel spins forward when the modules are in the zero position. if applying + to + and - to minus on the motor causes the wheel to spin backwards, set this to true.
    */
    SwerveModule(uint8_t driveMotorPort,
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
                 bool driveInversion = false);
    /**
     * Initialize the module. This should be called in the setup function of the main program after the global seesaw and interrupt line
     * have been configured, and before any calls to driveModule. This will zero the steering encoder using the configured zero switch and
     * home position, so it WILL CAUSE THE MODULE TO ROTATE.
     */
    void initializeModule();
    /**
     * Drive the module using a robot-relative target angle. Automatically chooses the shortest path to the target angle and reverses drive
     * direction if beneficial.
     *
     * @param targetAngle Desired robot-relative steering angle in degrees. 0 is the current forward direction of the module, positive is CCW.
     * @param driveSpeed Desired non-directional drive scalar speed.
     */
    void driveModule(float targetAngle, float driveSpeed);

    /**
     * Drive the module using a module-relative target angle. Does not perform any optimization and always drives in the commanded direction.
     *
     * @param targetAngle Desired module-relative steering angle in degrees.
     * @param driveVelocity Desired directional drive velocity command. Positive values correspond to the "forward" direction of the module,
     *  and negative values correspond to the "reverse" direction of the module, regardless of the actual steering angle.
     */
    void directDriveModule(float targetAngle, float driveVelocity);

    /**
     * Get the current robot-relative module angle.
     *
     * @return Current module steering angle in degrees.
     */
    float getModuleAngle();

    /**
     * Get the current module drive velocity.
     *
     * @return Current wheel speed estimate.
     */
    float getModuleVelocity();

    /**
     * Set brake mode for the drive motor.
     *
     * @param brake True to enable braking, false to coast.
     */
    void setBrakeMode(bool brake);
    /**
     * Update the module state. Should be called periodically to update the module's internal state.
     */
    void updateModuleState();
    /**
     * Get direct access to the drive motor object.
     *
     * @return Pointer to the internal drive motor (never nullptr).
     */
    NoU_Motor *getDriveMotor();

    /**
     * Get direct access to the steering motor object.
     *
     * @return Pointer to the internal steering motor (never nullptr).
     */
    NoU_Motor *getTurnMotor();
    /**
     * Get direct access to the steering encoder object.
     *
     * @return Pointer to the internal steering encoder (never nullptr).
     */
    // QuicEncoder *getTurnEncoder();
    /**
     * Get a copy of this module object.
     *
     * @return Copy of the current SwerveModule instance.
     */
    SwerveModule getModule();

private:
    NoU_Motor driveMotor;
    NoU_Motor turnMotor;
    UniversalEncoder *turnEncoder;
    float driveGearRatio;
    float turnGearRatio;
    float encoderGearRatio;
    float currentSpeed;
    float currentAngle, encoderHomePosition;
    bool turnInversion;
    bool driveInversion;

};
class UniversalEncoder
{
public:
    /**
     * Constructor
     *    @param getPositionFunc Function to get the current encoder position. as a float in degrees. This should account for any necessary
     *  gear ratios, offsets, and inversions to return the actual module angle.
     *    @param setPositionFunc Function to set the encoder position. This is used to zero the encoder during initialization by setting the
     *  current position to the known home position. It should accept a float in degrees and convert it to the appropriate units for the
     * underlying encoder implementation.
     *    @param updateFunc Function to update the encoder state. This is called periodically to update the internal state of the encoder.
     *    @param zeroSwitchSupplier Function that returns true when the encoder's zero switch is triggered. This is used during initialization
     *  to find the zero position of the module. It should return a boolean indicating whether the zero switch is currently active.
     */
    UniversalEncoder(std::function<float()> getPositionFunc, 
    std::function<void(float)> setPositionFunc,
     std::function<void()> updateFunc = []() {},
      std::function<bool()> zeroSwitchSupplier);
      
    std::function<float()> getPosition;
    std::function<void(float)> setPosition;
    std::function<void()> update;
    std::function<bool()> zeroSwitch;
};
// class QuicEncoder
// {
// public:
//     /**
//      * Construct a quadrature encoder reader that pulls pin state from the configured input source.
//      *
//      * @param pinA The first encoder channel. In seesaw mode, this is the seesaw GPIO pin number.
//      * @param pinB The second encoder channel. In seesaw mode, this is the seesaw GPIO pin number.
//      * @param inverted Whether to invert the encoder direction. Set so that it is CCW+ for correct modules.
//      */
//     QuicEncoder(uint8_t pinA, uint8_t pinB, bool inverted = false);
//     /**
//      * Initialize the encoder. The encoder uses the globally configured seesaw and interrupt line
//      * when those are set before construction.
//      */
//     void initialize();
//     /**
//      * Get the current position of the encoder.
//      *
//      * @return Current encoder position.
//      */
//     int32_t getPosition();
//     /**
//      * Reset the encoder position.
//      *
//      * @param newPosition The new position to set. Defaults to 0 if not specified.
//      */
//     void resetPosition(int32_t newPosition = 0);
//     /**
//      * Update the encoder state using the current GPIO values and the existing quadrature transition table.
//      * This is the same decode path used for direct pins and seesaw-backed pins.
//      */
//     void update();

// private:
//     uint8_t pinA, pinB;
//     bool inverted;
//     volatile uint8_t prevState;
//     volatile int32_t position;
// };
#endif