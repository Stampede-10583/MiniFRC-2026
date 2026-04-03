#ifndef MiniSwerveLib_h
#define MiniSwerveLib_h
#include "Arduino.h"
#include <vector>
#include "Alfredo_NoU3.h"
#include "Alfredo_NoU3_encoder.h"
class SwerveDrive
{
public:
    /**
     * Construct a swerve drive system with module and robot configuration data.
     *
     * @param driveMotorPorts Drive motor controller ports in module order {FL, FR, BL, BR}.
     * @param driveMotorInversions Per-module drive motor inversion flags. Set so that it is CCW+ (forward) for all modules.
     * @param driveGearRatios Per-module drive gear ratios.
     * @param turnServoPorts Turn servo controller ports in module order {FL, FR, BL, BR}.
     * @param turnServoOffsets Per-module steering angle offsets in degrees.
     * @param turnServoInversions Per-module turn servo inversion flags. Set so that it is CCW+ for all modules.
     * @param turnServoGearRatios Per-module turn gear ratios.
     * @param turnServoDeadspots Per-module steering deadspot values in degrees.
     * @param servoConfig Array containing {servoMinPulse (corresponds to 0°), servoMaxPulse (corresponds to 180°)} PWM bounds for steering servos. Defaults to {540, 2300} which is a common range 9g servo.
     * @param imuAngularScale Scale factor applied to IMU angular readings.
     * @param maxSpeed Maximum commanded linear speed.
     */
    SwerveDrive(uint8_t driveMotorPorts[4],
                std::array<bool, 4> driveMotorInversions = {false, false, false, false},
                std::array<float, 4> driveGearRatios = {1, 1, 1, 1}, uint8_t turnServoPorts[4],
                std::array<float, 4> turnServoOffsets = {0, 0, 0, 0},
                std::array<bool, 4> turnServoInversions = {false, false, false, false},
                std::array<float, 4> turnServoGearRatios = {1, 1, 1, 1}, std::array<float, 4> turnServoDeadspots = {0, 0, 0, 0},
                std::array<uint16_t, 2> servoConfig = {540, 2300},
                float imuAngularScale,
                float maxSpeed);
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
     * @param wrapendpoint Steering wrap limit in degrees.
     * @param drivegearratio Drive gear ratio from motor to wheel.
     * @param turngearratio Steering gear ratio from servo to module.
     * @param servoConfig Array containing {servoMinPulse (corresponds to 0°), servoMaxPulse (corresponds to 180°)} PWM bounds for steering servos. Defaults to {540, 2300} which is a common range 9g servo.
     * @param brakeMode True to enable drive motor braking.
     */
    SwerveModule(uint8_t driveServoPort, bool driveMotorInversion = false, uint8_t turnServoPort, bool turnServoInversion = false, float angleoffset, float wrapendpoint, float drivegearratio, float turngearratio, std::array<uint16_t, 2> servoConfig = {540, 2300}, bool brakeMode = false);

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
     * Get a copy of this module object.
     *
     * @return Copy of the current SwerveModule instance.
     */
    SwerveModule getModule();

private:
    NoU_Motor driveMotor;
    NoU_Servo turnServo;
    Encoder driveEncoder;
    float angleOffset;
    float wrapEndpoint;
    float driveGearRatio;
    float turnGearRatio;
    float currentSpeed;
    float currentAngle;
    bool turnInversion;
};
#endif