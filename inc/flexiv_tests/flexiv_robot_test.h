#pragma once

#include "core/ITest.h"
#include "core/data_logger.h"
#include <flexiv/rdk/robot.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <array>
#include <vector>
#include <Eigen/Geometry>

// Enum for all loggable data groups from Flexiv robot
enum class LogField {
    // Measured values from robot
    TCP_POSE,           // 7 values: x, y, z, qw, qx, qy, qz
    TCP_VEL,            // 6 values: vx, vy, vz, wx, wy, wz
    JOINT_POS,          // 7 values: j1-j7 in radians
    JOINT_VEL,          // 7 values: dj1-dj7 in rad/s
    EXT_WRENCH_TCP,     // 6 values: fx, fy, fz, mx, my, mz (in TCP frame)
    EXT_WRENCH_WORLD,   // 6 values: fx, fy, fz, mx, my, mz (in world frame)
    FT_SENSOR_RAW,      // 6 values: fx, fy, fz, mx, my, mz

    // Commanded values (for tests that track commands)
    CMD_TCP_POSE,       // 7 values: commanded TCP pose
    CMD_TCP_VEL,        // 6 values: commanded TCP velocity
    CMD_JOINT_POS,      // 7 values: commanded joint positions

    // Error values (for admittance and impedance control)
    POSE_ERROR,         // 3 values: position error (x, y, z) in meters
    ORI_ERROR           // 3 values: orientation error (ex, ey, ez) in radians
};

class FlexivRobotTest : public ITest{

public:
    FlexivRobotTest(std::string testName, const std::string& robotSn);
    virtual ~FlexivRobotTest();

    bool initialise() override;
    TestResult runTest() override;
    void stop() override;
    void cleanup() override;
    bool isFinished() const { return testFinished_; }
    void waitForCompletion();

protected:
    std::string testName_;
    std::string robotSn_;
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> testFinished_{true};
    std::unique_ptr<flexiv::rdk::Robot> robot_;
    std::thread thread_;
    std::mutex mtx_;
    DataLogger* logger_{nullptr};

    // Logging configuration
    std::vector<LogField> logFields_;
    std::array<double, 7> cmdTcpPose_{};      // Commanded TCP pose (optional)
    std::array<double, 6> cmdTcpVel_{};       // Commanded TCP velocity (optional)
    std::vector<double> cmdJointPos_;         // Commanded joint positions (optional)
    std::array<double, 3> poseError_{};       // Position error (optional)
    std::array<double, 3> oriError_{};        // Orientation error (optional)

    virtual void performTest() = 0;

    // New logging interface with field selection
    void startLogging(const std::string& filename,
                      const std::vector<LogField>& fields,
                      int intervalMs = 1);

    // Legacy logging interface (for backward compatibility)
    void startLogging(const std::string& filename, int intervalMs = 1);

    void stopLogging();

    // Set commanded values for logging (optional - only used if CMD_* fields are selected)
    void setCommandedTcpPose(const std::array<double, 7>& pose);
    void setCommandedTcpVel(const std::array<double, 6>& vel);
    void setCommandedJointPos(const std::vector<double>& pos);

    // Set error values for logging (optional - only used if *_ERROR fields are selected)
    void setPoseError(const std::array<double, 3>& error);
    void setOriError(const std::array<double, 3>& error);

    // Utility functions for unit conversions
    static double mmToM(double mm);
    static double mToMM(double m);
    static double degToRad(double deg);
    static double radToDeg(double rad);

    // Utility functions for pose conversions
    static std::array<double, 4> eulerZYXToQuat(const std::array<double, 3>& euler_rad);
    static std::array<double, 3> quatToEulerZYX(const std::array<double, 4>& quat);

    // Convert pose from [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] to [x_m, y_m, z_m, qw, qx, qy, qz]
    static std::array<double, 7> convertPose_mmDeg_to_mQuat(const std::array<double, 6>& pose_mm_deg);

    // Convert pose from [x_m, y_m, z_m, qw, qx, qy, qz] to [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    static std::array<double, 6> convertPose_mQuat_to_mmDeg(const std::array<double, 7>& pose_m_quat);

    // Check if target Cartesian pose has been reached
    // target_pose_mm_deg: {x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg}
    // axes_to_check: {x, y, z, rx, ry, rz} - true to check that axis, false to ignore
    // threshold_m: position threshold in meters (default 5mm)
    // threshold_deg: orientation threshold in degrees (default 5 deg)
    bool isCartesianPoseReached(const std::array<double, 6>& target_pose_mm_deg,
                                const std::array<bool, 6>& axes_to_check = {true,true,true,true,true,true},
                                double threshold_m = 0.005,
                                double threshold_deg = 5.0);

    // Check if target joint pose has been reached
    // target_joint_pos_deg: {j1_deg, j2_deg, j3_deg, j4_deg, j5_deg, j6_deg, j7_deg}
    // joints_to_check: {j1, j2, j3, j4, j5, j6, j7} - true to check that joint, false to ignore
    // threshold_deg: joint angle threshold in degrees (default 1 deg)
    bool isJointPoseReached(const std::array<double, 7>& target_joint_pos_deg,
                            const std::array<bool, 7>& joints_to_check = {true,true,true,true,true,true,true},
                            double threshold_deg = 1.0);


    void FlexivRobotTest::ZeroFTSensor();

private:
    // Internal logging callback
    void logSelectedFields(std::ostream& out);

    // Helper to write field headers for CSV
    static void writeFieldHeaders(std::ostream& out, LogField field);

    // Helper to write field values
    void writeFieldValues(std::ostream& out, LogField field) const;

};
