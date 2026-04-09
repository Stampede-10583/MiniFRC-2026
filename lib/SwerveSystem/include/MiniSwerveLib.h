#ifndef MiniSwerveLib_h
#define MiniSwerveLib_h
#include <Arduino.h>
#include <array>
#include <vector>
#include <functional>
#include <Alfredo_NoU3.h>
#include <Alfredo_NoU3_encoder.h>
#include <QuickPID.h>
#include <VectorXf.h>

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
    bool isAbsolute, isStockEncoder; // for stock encoders, we can assume certain things about the behavior of the zero switch and the setPosition function, which allows us to simplify the initialization process in the module code. This should be set to true if using a stock encoder with a known zeroing procedure, and false if using a custom encoder implementation.
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
     * @param kPID kP, kI, kD, for the module position PID controller (look into PID Controllers for details)
     */
    SwerveModule(uint8_t driveMotorPort,
                 bool driveMotorInversion,
                 uint8_t turnMotorPort,
                 bool turnMotorInversion,
                 float drivegearratio,
                 float turngearratio,
                 UniversalEncoder* turnEncoder,
                 bool brakeMode,
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
     * @param kPID kP, kI, kD, for the module position PID controller (look into PID Controllers for details)
     */
    SwerveModule(uint8_t driveMotorPort,
                 bool driveMotorInversion,
                 NoU_Motor *turnMotor,
                 bool turnMotorInversion,
                 float drivegearratio,
                 float turngearratio,
                 UniversalEncoder* turnEncoder,
                 bool brakeMode,
                 std::array<float, 3> kPID);
    /**
     * Stop the module.
     */
    void stopModule()
    {
        driveMotor.set(0);
        turnMotor->set(0);
    }
    /**
     * Initialize the module. This should be called in the setup function of the main program
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
    NoU_Motor driveMotor;
    NoU_Motor *turnMotor;
    UniversalEncoder *turnEncoder;
    QuickPID posPIDController;
    float driveGearRatio, turnGearRatio, currentSpeed, currentAngle, PIDInput, turnOutput, PIDSetpoint, driveSetpoint;
    bool turnInversion, driveInversion, stockEncoder;
};
class SwerveDrive
{
public:
    /**
     * Construct a swerve drive system. Assuming you are using an INTERNAL NoU3 encoder for the turn angle.
     * FOR ALL ARRAYS:
     * ORDER IS {FL, FR, BL, BR}
     *
     * @param agent Pointer to the NoU_Agent instance to use for this drive. This is used to allow the drive to access the IMU data for field-oriented control if desired.
     * @param imuAngularScale The scale factor to apply to the raw IMU angle data. This is used to correct for any inaccuracies in the IMU readings and should be determined through testing and calibration.
     * @param driveMotorPorts Drive motor controller ports in module order {FL, FR, BL, BR}.
     * @param driveMotorInversions Per-module drive motor inversion flags. Set so that it is CCW+ (forward) for all modules.
     * @param driveGearRatios Per-module drive gear ratios.
     * @param turnMotors Turn motor controller instances in module order {FL, FR, BL, BR}.
     * @param turnMotorInversions Per-module turn motor inversion flags. Set so that it is CCW+ for all modules.
     * @param turnMotorGearRatios Per-module turn gear ratios.
     * @param turnEncoders Per-module steering encoders.
     * @param turnEncoderHomePositions Per-module home position values for the steering encoder.
     * @param maxSpeed Maximum commanded linear speed. This is used to cap the output of the drive functions and can be adjusted on the fly.
     * @param brakeMode True to enable drive motor braking, false for coast.
     * @param moduleOffsets Per-module offset positions from the robot's center. This is used for accurate field-oriented control and should be specified with units in meters.
     * @param kPID kP, kI, kD, for the module position PID controllers (look into PID Controllers for details)
     */
    SwerveDrive(NoU_Agent* agent,
                float imuAngularScale,
                uint8_t driveMotorPorts[4],
                std::array<bool, 4> driveMotorInversions,
                std::array<float, 4> driveGearRatios,
                std::array<NoU_Motor *, 4> turnMotors,
                std::array<bool, 4> turnMotorInversions,
                std::array<float, 4> turnMotorGearRatios,
                std::array<UniversalEncoder *, 4> turnEncoders,
                float maxSpeed,
                bool brakeMode,
                std::array<std::array<float, 2>, 4> moduleOffsets,
                std::array<float, 3> kPID);
    /**
     * Construct a swerve drive system. Assuming you are using an external encoder for the turn angle.
     * FOR ALL ARRAYS:
     * ORDER IS {FL, FR, BL, BR}
     *
     * @param agent Pointer to the NoU_Agent instance to use for this drive. This is used to allow the drive to access the IMU data for field-oriented control if desired.
     * @param imuAngularScale The scale factor to apply to the raw IMU angle data. This is used to correct for any inaccuracies in the IMU readings and should be determined through testing and calibration.
     * @param driveMotorPorts Drive motor controller ports in module order {FL, FR, BL, BR}.
     * @param driveMotorInversions Per-module drive motor inversion flags. Set so that it is CCW+ (forward) for all modules.
     * @param driveGearRatios Per-module drive gear ratios.
     * @param turnMotorPorts Turn motor controller ports in module order {FL, FR, BL, BR}.
     * @param turnMotorInversions Per-module turn motor inversion flags. Set so that it is CCW+ for all modules.
     * @param turnMotorGearRatios Per-module turn gear ratios.
     * @param turnEncoders Per-module steering encoders.
     * @param turnEncoderHomePositions Per-module home position values for the steering encoder.
     * @param maxSpeed Maximum commanded linear speed. This is used to cap the output of the drive functions and can be adjusted on the fly.
     * @param brakeMode True to enable drive motor braking, false for coast.
     * @param moduleOffsets Per-module offset positions from the robot's center. This is used for accurate field-oriented control and should be specified with units in meters.
     * @param kPID kP, kI, kD, for the module position PID controllers (look into PID Controllers for details)
     */
    SwerveDrive(NoU_Agent* agent,
                float imuAngularScale,
                uint8_t driveMotorPorts[4],
                std::array<bool, 4> driveMotorInversions,
                std::array<float, 4> driveGearRatios,
                uint8_t turnMotorPorts[4],
                std::array<bool, 4> turnMotorInversions,
                std::array<float, 4> turnMotorGearRatios,
                std::array<UniversalEncoder *, 4> turnEncoders,
                float maxSpeed,
                bool brakeMode,
                std::array<std::array<float, 2>, 4> moduleOffsets,
                std::array<float, 3> kPID);
    /**
     * Calculate target velocities for the robot based on joystick inputs.
     *
     * @param xJoystick X-axis joystick input. X=forward, -X=backward.
     * @param yJoystick Y-axis joystick input. Y=left, -Y=right.
     * @param rotationJoystick Rotation joystick input. Positive=CCW, Negative=CW.
     * @param scale Velocity scaling factor.
     * @param robotOriented Whether to use robot-oriented coordinate system.
     * @return Vector containing {x, y, angular} velocity components.
     */
    std::vector<float> getTargetVelocities(float xJoystick, float yJoystick, float rotationJoystick, float scale, bool robotOriented);
    /**
     * Drive all modules field oiented.
     *
     * @param velocities Desired robot velocity vector {x, y, angular} in m/s and deg/s. Generate this via getTargetVelocities() helper.
     * @param driveSpeed Desired robot speed in m/s. This is a scalar value.
     */
    void drive(std::vector<float> velocities, float driveSpeed);
    /**
     * Set explicit target angle and speed for each module.
     *
     * @param targetAngles Per-module steering targets in degrees.
     * @param driveSpeeds Per-module speed targets in m/s.
     */
    void driveModules(float targetAngles[4], float driveSpeeds[4]);
    /**
     * Set explicit motor outputs for each module, bypassing the PID controller. This is useful for testing and debugging, but should be used with caution during normal operation as it can lead to unexpected behavior if not used correctly.
     *
     * @param turnSpeeds Per-module steering motor outputs. Should be between -1 and 1, where the sign indicates direction and the magnitude indicates power level.
     * @param driveSpeeds Per-module drive motor outputs. Should be between -1 and 1, where the sign indicates direction and the magnitude indicates power level.
     */
    void directDriveModules(float turnSpeeds[4], float driveSpeeds[4]);

    /**
     * Drive toward a field target point.
     *
     * @param x Target X position.
     * @param y Target Y position.
     * @param targetAngle Target robot heading in degrees.
     * @param maxSpeed Speed cap to use for this maneuver.
     */
    void driveToPoint(float x, float y, float targetAngle, float maxSpeed);

    /**
     * Enable or disable brake mode on all modules.
     *
     * @param brake True to enable braking behavior, false to coast.
     */
    void setBrakeMode(bool brake);

    /**
     * Update the global maximum speed used by drive logic.
     *
     * @param maxSpeed New maximum speed value in m/s.
     */
    void setMaxSpeed(float maxSpeed);
    /**
     * Get the current estimated robot pose.
     *
     * @return Array containing {x, y, heading}.
     */
    std::array<float, 3> getOdeometry();
    /**
     * Reset odometry state to a specified pose.
     *
     * @param x New X position.
     * @param y New Y position.
     * @param angle New heading in degrees.
     */
    void resetOdeometry(float x, float y, float angle);
    /**
     * Run periodic swerve subsystem updates.
     */
    void swerveDrivePeriodic(bool robotEnabled);
    /**
     * Stop all swerve modules.
     */
    void stopAllModules()
    {
        for (int i = 0; i < 4; i++)
        {
            modules[i].stopModule();
        }
    }
    /**
     * Update the PID constants for all modules.
     *
     * @param kPID New PID constants {kP, kI, kD}.
     */
    void updatePIDConstants(std::array<float, 3> kPID)
    {
        for (int i = 0; i < 4; i++)
        {
            modules[i].getPIDController()->SetTunings(kPID[0], kPID[1], kPID[2]);
        }
    }
    /**
     * Get a pointer to a module by index.
     *
     * @param index Module index in range [0, 3]. 0=FL, 1=FR, 2=BL, 3=BR.
     * @return Pointer to the requested module, or nullptr if index is invalid.
     */
    SwerveModule *getModule(int index);
    /**
     * Get a copy of this drive object.
     *
     * @return Copy of the current SwerveDrive instance.
     */
    SwerveDrive getDrive();

private:
    NoU_Agent *agent;
    SwerveModule modules[4];
    std::array<std::array<float, 2>, 4> moduleOffsets;
    float imuAngularScale, maxSpeed, xPos, yPos, heading;
    bool brakeMode;
};
#endif