#ifndef SingleModuleTest_h
#define SingleModuleTest_h
#include "Arduino.h"
#include <array>
#include <vector>
#include <functional>
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
#include <QuickPID.h>

//class Adafruit_seesaw;
class UniversalEncoder
{
public:
    /**
     * Constructor for RELATIVE ENCODERS. Note that all Lambdas that deal with angles should take/return in a float of angles in degrees with CCW+. All gear ratios, inversions, and CPR conversions should be done by YOU before returning a value. See the docs for an example for a NoU3 Encoder
     *    @param getPositionFunc Function to get the current encoder position. as a float in degrees. This should account for any necessary
     *  gear ratios, offsets, and inversions to return the actual module angle.
     *    @param setPositionFunc Function to set the encoder position. This is used to zero the encoder during initialization by setting the
     *  current position to the known home position. It should accept a float in degrees and convert it to the appropriate units for the
     * underlying encoder implementation.
     *    @param updateFunc Function to update the encoder state. This is called periodically to update the internal state of the encoder. Do nothing if unnecesary
     *    @param zeroSwitchSupplier Function that returns true when the encoder's zero switch is triggered. This is used during initialization
     *  to find the zero position of the module. It should return a boolean indicating whether the zero switch is currently active.
     *    @param homePosition The angle in degrees CCW+ (see docs for info) of the MODULE when the zero switch is closed.
     */
    UniversalEncoder(bool isAbsolute, std::function<float()> getPositionFunc, std::function<void()> updateFunc, std::function<void(float)> setPositionFunc, std::function<bool()> zeroSwitchSupplier, float homePosition);
    /**
     * Constructor for ABSOLUTE ENCODERS. Note that getPosition() should return in a float of angles in degrees with CCW+. All gear ratios, inversions, and CPR conversions should be done by YOU before returning a value. See the docs for an psuedocode example.
     *    @param getPositionFunc Function to get the current encoder position. as a float in degrees. This should account for any necessary
     *  gear ratios, offsets, and inversions to return the actual module angle.
     *    @param updateFunc Function to update the encoder state. This is called periodically to update the internal state of the encoder. Do nothing if unnecesary
     */
    UniversalEncoder(std::function<float()> getPositionFunc, std::function<void()> updateFunc);

    std::function<float()> getPosition;
    std::function<void(float)> setPosition;
    std::function<void()> update;
    std::function<bool()> zeroSwitch;
    float homePosition;
private:
    bool isAbsolute;
    bool isStockEncoder; // for stock encoders, we can assume certain things about the behavior of the zero switch and the setPosition function, which allows us to simplify the initialization process in the module code. This should be set to true if using a stock encoder with a known zeroing procedure, and false if using a custom encoder implementation.

};
class SwerveModule
{
public:
    bool initialized = false;
    /**
     * FOR NON-BUILT IN TURN ENCODERS Construct a swerve module controller.
     *
     * @param driveMotorPort Port on the NoU3 for the drive motor controller.
     * @param driveMotorInversion Set this so that the drive wheel spins forward when the modules are in the zero position. If applying standard current direction causes the wheel to spin backwards, set this to true.
     * @param turnMotorPort Port on the NoU3 for the steering motor.
     * @param turnMotorInversion True to invert steering direction. If applying standard direction of current yeilds in CW rotation of the module, set this to true.
     * @param drivegearratio Drive gear ratio from motor to wheel.
     * @param turngearratio Steering gear ratio from motor to module.
     * @param turnEncoder UniversalEncoder instance for the steering encoder. This is used to allow flexible configuration of the encoder implementation while keeping the module code generic.
     * @param brakeMode True to enable drive motor braking.
     * @param agent Pointer to the NoU_Agent instance to use for this module. This is used to allow the module to access the IMU data for field-oriented control if desired, and also allows for more flexible configuration of the NoU_Agent instance used by the module.
     * @param kPID kP, kI, kD, for the module position PID controller (look into PID Controllers for details)
     */
    SwerveModule(uint8_t driveMotorPort,
                 bool driveMotorInversion,
                 uint8_t turnMotorPort,
                 bool turnMotorInversion,
                 float drivegearratio,
                 float turngearratio,
                 UniversalEncoder *turnEncoder,
                 bool brakeMode,
                NoU_Agent *agent,
                std::array<float, 3> kPID);
    /**
     * FOR BUILT IN TURN ENCODERS Construct a swerve module controller.
     *
     * @param driveMotorPort Port on the NoU3 for the drive motor controller.
     * @param driveMotorInversion Set this so that the drive wheel spins forward when the modules are in the zero position. If applying standard current direction causes the wheel to spin backwards, set this to true.
     * @param turnMotor Steering motor ptr. this allows you to access the encoder for making a custom UniversalEncoder implementation if you are using a built in encoder on the motor, and also allows for more flexible motor controller options for the steering motor.
     * @param turnMotorInversion True to invert steering direction. If applying standard direction of current yeilds in CW rotation of the module, set this to true.
     * @param drivegearratio Drive gear ratio from motor to wheel.
     * @param turngearratio Steering gear ratio from motor to module.
     * @param turnEncoder UniversalEncoder instance for the steering encoder. This is used to allow flexible configuration of the encoder implementation while keeping the module code generic.
     * @param brakeMode True to enable drive motor braking.
     * @param agent Pointer to the NoU_Agent instance to use for this module. This is used to allow the module to access the IMU data for field-oriented control if desired, and also allows for more flexible configuration of the NoU_Agent instance used by the module.
     * @param kPID kP, kI, kD, for the module position PID controller (look into PID Controllers for details)
     */
    SwerveModule(uint8_t driveMotorPort,
                 bool driveMotorInversion,
                 NoU_Motor *turnMotor,
                 bool turnMotorInversion,
                 float drivegearratio,
                 float turngearratio,
                 UniversalEncoder *turnEncoder,
                 bool brakeMode,
                NoU_Agent *agent,
                std::array<float, 3> kPID);
    /**
     * Stop the module.
     */
    void stopModule() {
        driveMotor.set(0);
        turnMotor->set(0);
    }
    /**
     * Initialize the module. This should be called in the setup function of the main program after the global seesaw and interrupt line
     * have been configured, and before any calls to driveModule. This will zero the steering encoder using the configured zero switch and
     * home position, so it WILL CAUSE THE MODULE TO ROTATE.
     * @return True if initialized, false if not initialized.
     * Note that this function will block until the zero switch is triggered, so it should only be called once during setup and not called again during operation.
     */
    bool initializeModule();
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
     * Run in loop whenever the bot is "Enabled" i.e. whenever you wish for it to move
     */
    void driveMotors();
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
    UniversalEncoder *getTurnEncoder();
    /**
     * Get a copy of this module object.
     *
     * @return Copy of the current SwerveModule instance.
     */
    SwerveModule getModule();
    /**
     * Get a pointer to the Quick_PID object that is in use
     * @returns Pointer to Quick_PID object
     */
    QuickPID *getPIDController();
private:
    NoU_Agent *agent;
    NoU_Motor driveMotor;
    NoU_Motor *turnMotor;
    UniversalEncoder *turnEncoder;
    QuickPID posPIDController;
    float driveGearRatio, turnGearRatio, currentSpeed, currentAngle, PIDInput, turnOutput, PIDSetpoint, driveSetpoint;
    bool turnInversion, driveInversion, stockEncoder;
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