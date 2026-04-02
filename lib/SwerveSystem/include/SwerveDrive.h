#ifndef Morse_h
#define Morse_h
#include "Arduino.h"
#include "SwerveModule.h"
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
     * @param imuAngularScale Scale factor applied to IMU angular readings.
     * @param maxSpeed Maximum commanded linear speed.
     */
    SwerveDrive(uint8_t driveMotorPorts[4], std::array<bool, 4> driveMotorInversions = {false, false, false, false}, std::array<float, 4> driveGearRatios = {1, 1, 1, 1}, uint8_t turnServoPorts[4], std::array<float, 4> turnServoOffsets = {0, 0, 0, 0}, std::array<bool, 4> turnServoInversions = {false, false, false, false}, std::array<float, 4> turnServoGearRatios = {1, 1, 1, 1}, std::array<float, 4> turnServoDeadspots = {0, 0, 0, 0}, float imuAngularScale, float maxSpeed);

    /**
     * Drive all modules field oiented. 
     *
     * @param targetAngle Desired robot heading angle in degrees. 0 is forward, positive is CCW.
     * @param driveSpeed Desired robot speed in m/s. This is a scalar value.
     */
    void drive(float targetAngle, float driveSpeed);

    /**
     * Drive using robot-relative input coordinates. 
     *
     * @param targetAngle Desired motion angle in robot frame, in degrees.
     * @param driveSpeed Desired robot-relative speed in m/s.
     */
    void driveRobotOriented(float targetAngle, float driveSpeed);

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
    SwerveModule* getModule(int index);

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
#endif