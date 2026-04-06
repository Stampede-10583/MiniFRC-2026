#ifndef MiniSwerveLib_h
#define MiniSwerveLib_h
#include "Arduino.h"
#include <array>
#include <vector>
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
class Adafruit_seesaw;
class SwerveDrive
{
public:
    /**
     * Construct a swerve drive system and automatically configure the external encoder seesaw.
     *
     * @param driveMotorPorts Drive motor controller ports in module order {FL, FR, BL, BR}.
     * @param driveMotorInversions Per-module drive motor inversion flags. Set so that it is CCW+ (forward) for all modules.
     * @param driveGearRatios Per-module drive gear ratios.
     * @param turnServoPorts Turn servo controller ports in module order {FL, FR, BL, BR}.
     * @param turnServoOffsets Per-module steering angle offsets in degrees.
     * @param turnServoInversions Per-module turn servo inversion flags. Set so that it is CCW+ for all modules.
     * @param turnServoGearRatios Per-module turn gear ratios.
     * @param servoConfig Array containing {servoMinPulse (corresponds to 0°), servoMaxPulse (corresponds to 180°)} PWM bounds for steering servos. Defaults to {540, 2300} which is a common range 9g servo.
     * @param turnEncoderAndZeroSwitchPorts Per-module seesaw GPIO pin triplet containing {pinA, pinB, ClickPin} for the steering encoder and a switch that is triggered when the modue is at the home position. These are read directly from the external breakout board.
     * @param turnEncoderHomePositions Per-module home position values for the steering encoder.
     * @param turnEncoderInversions Per-module turn encoder inversion flags. Set so that it is CCW+ for all modules.
     * @param encoderSeesaw Initialized seesaw object used to read the external encoder GPIO pins.
     * The drive assumes the seesaw interrupt line is wired to ESP GPIO 9.
     * @param imuAngularScale Scale factor applied to IMU angular readings.
     * @param maxSpeed Maximum commanded linear speed.
     * @param brakeMode True to enable drive motor braking, false for coast.
     * @param interruptPin MCU pin wired to the seesaw interrupt output. This will be 8 or 9 on NoU3. Leave empty to disable interrupt (passes -1 internally).
     */
    SwerveDrive(uint8_t driveMotorPorts[4],
                std::array<bool, 4> driveMotorInversions = {false, false, false, false},
                std::array<float, 4> driveGearRatios = {1, 1, 1, 1}, uint8_t turnServoPorts[4],
                std::array<float, 4> turnServoOffsets = {0, 0, 0, 0},
                std::array<bool, 4> turnServoInversions = {false, false, false, false},
                std::array<float, 4> turnServoGearRatios = {1, 1, 1, 1},
                std::array<uint16_t, 2> servoConfig = {540, 2300},
                std::array<uint8_t, 4> turnEncoderAndZeroSwitchPorts = {0, 1, 2, 3},
                std::array<int32_t, 4> turnEncoderHomePositions = {0, 0, 0, 0},
                std::array<bool, 4> turnEncoderInversions = {false, false, false, false},
                Adafruit_seesaw *encoderSeesaw,
                float imuAngularScale,
                float maxSpeed, 
                bool brakeMode = false, 
                uint8_t interruptPin = -1);
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
    void setModuleStates(float targetAngles[4], float driveSpeeds[4]);

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
     * Update the IMU angular scale factor.
     *
     * @param imuAngularScale New IMU scale factor.
     */
    void setIMUScale(float imuAngularScale);

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
    void swerveDrivePeriodic();
    /**
     * Configure the seesaw for encoder operation.
     *
     * @param seesaw Pointer to the Adafruit_seesaw instance.
     * @param interruptPin MCU pin wired to the seesaw interrupt output. This will be 8 or 9 on NoU3. Pass a negative value to disable the interrupt.
     */
    static void configureSeesaw(Adafruit_seesaw *seesaw, int8_t interruptPin);
    /**
     * Configure the MCU interrupt pin connected to seesaw INT.
     *
     * @param interruptPin MCU pin wired to the seesaw interrupt output. Pass a negative value to disable.
     */
    static void configureSeesawInterruptPin(int8_t interruptPin);
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
    SwerveModule modules[4];
    float imuAngularScale;
    float maxSpeed;
};
class SwerveModule
{
public:
    /**
     * Construct a swerve module controller.
     *
     * @param driveServoPort Port for the drive motor controller.
     * @param driveMotorInversion True to invert the drive motor direction.
     * @param turnServoPort Port for the steering servo.
     * @param turnServoInversion True to invert steering direction.
     * @param angleoffset Steering zero offset in degrees.
     * @param drivegearratio Drive gear ratio from motor to wheel.
     * @param turngearratio Steering gear ratio from servo to module.
     * @param encodergearratio Encoder gear ratio for position feedback.
     * @param servoConfig Array containing {servoMinPulse (corresponds to 0°), servoMaxPulse (corresponds to 180°)} PWM bounds for steering servos. Defaults to {540, 2300} which is a common range 9g servo.
     * @param turnEncoderPins Seesaw GPIO pin pair containing {pinA, pinB, ClickPin} for the steering encoder and a switch that is triggered when the modue is at the home position.
     * These are read directly from the external breakout board.
     * @param turnEncoderHomePosition The encoder position corresponding to the home position in degrees. This is used to zero the encoder during initialization.
     * @param turnEncoderInversions Per-module turn encoder inversion flags. Set so that it is CCW+ for all modules.
     * @param brakeMode True to enable drive motor braking.
     */
    SwerveModule(uint8_t driveServoPort,
                 bool driveMotorInversion = false,
                 uint8_t turnServoPort,
                 bool turnServoInversion = false,
                 float angleoffset,
                 float drivegearratio,
                 float turngearratio,
                 float encodergearratio,
                 std::array<uint16_t, 2> servoConfig = {540, 2300},
                 uint8_t turnEncoderAndZeroSwitchPort,
                 int32_t turnEncoderHomePosition,
                 bool turnEncoderInversion = false,
                 bool brakeMode = false);
    
    void initializeModule();
    /**
     * Drive the module using a robot-relative target angle. Automatically chooses the shortest path to the target angle and reverses drive direction if beneficial.
     *
     * @param targetAngle Desired robot-relative steering angle in degrees. 0 is the current forward direction of the module, positive is CCW.
     * @param driveSpeed Desired non-directional drive scalar speed.
     */
    void driveModule(float targetAngle, float driveSpeed);

    /**
     * Drive the module using a module-relative target angle. Does not perform any optimization and always drives in the commanded direction.
     *
     * @param targetAngle Desired module-relative steering angle in degrees.
     * @param driveVelocity Desired directional drive velocity command. Positive values correspond to the "forward" direction of the module, and negative values correspond to the "reverse" direction of the module, regardless of the actual steering angle.
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
     * Get direct access to the steering servo object.
     *
     * @return Pointer to the internal steering servo (never nullptr).
     */
    NoU_Servo *getTurnServo();
    /**
     * Get direct access to the steering encoder object.
     *
     * @return Pointer to the internal steering encoder (never nullptr).
     */
    QuicEncoder *getTurnEncoder();
    /**
     * Get a copy of this module object.
     *
     * @return Copy of the current SwerveModule instance.
     */
    SwerveModule getModule();

private:
    NoU_Motor driveMotor;
    NoU_Servo turnServo;
    QuicEncoder turnEncoder;
    float angleOffset;
    float driveGearRatio;
    float turnGearRatio;
    float encoderGearRatio;
    float currentSpeed;
    uint8_t encoderPinA, encoderPinB, zeroSwitchPin;
    int32_t currentAngle, encoderHomePosition;
    bool turnInversion;
};
class QuicEncoder
{
public:
    /**
     * Construct a quadrature encoder reader that pulls pin state from the configured input source.
     *
     * @param pinA The first encoder channel. In seesaw mode, this is the seesaw GPIO pin number.
     * @param pinB The second encoder channel. In seesaw mode, this is the seesaw GPIO pin number.
     * @param inverted Whether to invert the encoder direction. Set so that it is CCW+ for correct modules.
     */
    QuicEncoder(uint8_t pinA, uint8_t pinB, bool inverted = false);
    /**
     * Initialize the encoder. The encoder uses the globally configured seesaw and interrupt line
     * when those are set before construction.
     */
    void initialize();
    /**
     * Get the current position of the encoder.
     *
     * @return Current encoder position.
     */
    int32_t getPosition();
    /**
     * Reset the encoder position.
     *
     * @param newPosition The new position to set. Defaults to 0 if not specified.
     */
    void resetPosition(int32_t newPosition = 0);
    /**
     * Update the encoder state using the current GPIO values and the existing quadrature transition table.
     * This is the same decode path used for direct pins and seesaw-backed pins.
     */
    void update();

private:
    uint8_t pinA, pinB;
    bool inverted;
    volatile uint8_t prevState;
    volatile int32_t position;
};
#endif