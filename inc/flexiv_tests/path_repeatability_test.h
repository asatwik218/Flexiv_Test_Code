#ifndef PATH_REPEATABILITY_TEST_H
#define PATH_REPEATABILITY_TEST_H

#include "flexiv_tests/flexiv_robot_test.h"
#include <vector>
#include <array>

/**
 * @brief Test for checking path repeatability by executing point-to-point motion
 * through a sequence of waypoints.
 *
 * Waypoints are defined in camera_robot_base frame and transformed to world frame.
 */
class PathRepeatabilityTest : public FlexivRobotTest
{
public:
    /**
     * @brief Construct a new Path Repeatability Test object
     * @param robotSn Robot serial number
     */
    explicit PathRepeatabilityTest(const std::string& robotSn);

    void performTest() override;

private:
    // Conversion utilities
    inline double mmToM(double mm) const { return mm * 0.001; }
    inline double mToMM(double m) const { return m * 1000.0; }
    inline double degToRad(double deg) const;
    inline double radToDeg(double rad) const;

    /**
     * @brief Convert Euler ZYX angles to quaternion
     * @param euler [roll_rad, pitch_rad, yaw_rad]
     * @return std::array<double, 4> [qw, qx, qy, qz]
     */
    std::array<double, 4> eulerZYXToQuat(const std::array<double, 3>& euler) const;

    /**
     * @brief Convert quaternion to Euler ZYX angles
     * @param quat [qw, qx, qy, qz]
     * @return std::array<double, 3> [roll_rad, pitch_rad, yaw_rad]
     */
    std::array<double, 3> quatToEulerZYX(const std::array<double, 4>& quat) const;

    /**
     * @brief Transform pose from reference frame to world frame
     * @param poseInRefFrame_mm_deg Pose in reference frame [x, y, z, roll, pitch, yaw] in mm/deg
     * @param refCoordPose_mm_deg Reference frame pose in world [x, y, z, roll, pitch, yaw] in mm/deg
     * @param poseInWorld_mm_deg Output pose in world frame [x, y, z, roll, pitch, yaw] in mm/deg
     * @return int 0 on success, 1 on error
     */
    int transformPoseFromRefToWorld(
        const std::array<double, 6>& poseInRefFrame_mm_deg,
        const std::array<double, 6>& refCoordPose_mm_deg,
        std::array<double, 6>& poseInWorld_mm_deg
    ) const;

    /**
     * @brief Convert waypoint from mm/deg to m/quat format
     * @param waypoint_mm_deg [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]
     * @return std::array<double, 7> [x_m, y_m, z_m, qw, qx, qy, qz]
     */
    std::array<double, 7> convertWaypoint(const std::array<double, 6>& waypoint_mm_deg) const;
};

#endif // PATH_REPEATABILITY_TEST_H
