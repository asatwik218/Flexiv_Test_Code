#pragma once

#include "flexiv_tests/flexiv_robot_test.h"
#include <array>
#include <vector>

class PayloadTest : public FlexivRobotTest
{
public:
    explicit PayloadTest(const std::string& robotSn);
    ~PayloadTest() override = default;

protected:
    void performTest() override;
    void log(std::ostream& out) override;

private:
    // Core test execution methods
    void jointMotionTest(int testIndex, double payloadKg);
    void cartesianMotionTest(int testIndex, double payloadKg);
    void rotationalMotionTest(int testIndex, double payloadKg);
    void sequentialMotionTest(double payloadKg);

    // Helper functions for motion completion
    void waitForJointMotionComplete(const std::vector<double>& targetPose, const std::string& phase);
    void waitForCartesianMotionComplete(const std::vector<double>& targetPose, const std::string& phase);

    // Thread-safe state for logging (protected by inherited mtx_)
    struct TestState {
        std::string currentTest{};
        double currentPayload{0.0};
    };
    TestState lastTestState_;

    // Store discovered maximum safe accelerations for each Cartesian test (linear motion)
    // Index corresponds to cartesianTestPoses_ index
    std::vector<double> discoveredCartesianAccelerations_;

    // Store discovered maximum safe angular accelerations for rotational tests
    // Index corresponds to rotationalTestPoses_ index
    std::vector<double> discoveredAngularAccelerations_;

    // Test configuration
    std::vector<double> payloadWeights_ = { 6};  // kg

    // Joint limits and max velocities (degrees/sec for joints)
    // J1-J7 max velocities in deg/s
    std::array<double, 7> maxJointVelocities_ = {120, 120, 140, 140, 280, 280, 280};
    // J1-J7 max accelerations in deg/s^2
    std::array<double, 7> maxJointAccelerations_ = {900, 100, 900, 100, 200 , 1400, 1400};


    // Joint test poses - pairs of complete 7-joint configurations (DEGREES)
    // Each entry is: {initial_7_joint_pose, final_7_joint_pose}
    // Format: vector of pairs, where each pair contains two 7-element vectors
    // NOTE: Values are in DEGREES and will be converted to radians internally
    std::vector<std::pair<std::vector<double>, std::vector<double>>> jointTestPoses_ = {
        // Test 1: J1 motion
        {{-160,-50,-4,82,8,131,0}, {+160,-50,-4,82,8,131,0}},
        // Test 2: J2 motion
        {{0,-80,-4,82,8,131,0}, {0,130,-4,82,8,131,0}},
        // Test 3: J3 motion
        {{0, -90, -170, 0, 0, 90, 0}, {0, -90, 170, 0, 0, 90, 0}},
        // Test 4: J4 motion
        {{0,0,0,-107,0,90,0}, {0,0,0,154,0,90,0}},
        // Test 5: J5 motion
        {{0,0,0,90,-170,90,0}, {0,0,0,90,170,90,0}},
        // Test 6: J6 motion
        {{0,0, 0, 90, 0, -80, 0}, {0,0, 0, 90, 0, 200, 0}},
        // Test 7: J7 motion
        {{0,0,0,0,0,0,-170}, {0,0,0,0,0,0,170}}
    };

    // Cartesian test poses - pairs of complete TCP poses (X, Y, Z in meters, Rx, Ry, Rz in degrees)
    // Each entry is: {initial_6DOF_pose, final_6DOF_pose}
    // Format: vector of pairs, where each pair contains two 6-element vectors
    // NOTE: Linear motion tests only (X, Y, Z axes)
    std::vector<std::pair<std::vector<double>, std::vector<double>>> cartesianTestPoses_ = {
        // Test 1: X-axis motion
        {{0.755,-0.112,0.541,0,90,0}, {0.400,-0.112,0.541,0,90,0}},
        // Test 2: Y-axis motion
        {{0.755, -0.350, 0.293, 0,90,0}, {0.755, 0.200, 0.293, 0,90,0}},
        // Test 3: Z-axis motion
        {{0.755,-0.112,0.160,0,90,0}, {0.755,-0.112,0.850,0,90,0}},
    };

    // Rotational test poses - pairs of complete TCP poses (X, Y, Z in meters, Rx, Ry, Rz in degrees)
    // Each entry is: {initial_6DOF_pose, final_6DOF_pose}
    // Format: vector of pairs, where each pair contains two 6-element vectors
    // NOTE: Rotational motion tests only (Rx, Ry, Rz axes)
    std::vector<std::pair<std::vector<double>, std::vector<double>>> rotationalTestPoses_ = {
        // Test 1: Rx rotation
        {{0.700, -0.112, 0.500, 0, 180 , 0}, {0.700,-0.112,0.541, 180, -180,0}},
        // Test 2: Ry rotation
        {{0.700,-0.112,0.50,0,-130,0}, {0.755,-0.112,0.541,0,90,0}},
    };

    // Max TCP velocity (m/s for linear, rad/s for angular)
    double maxTcpLinearVel_ = 1.0;        // m/s
    double maxTcpAngularVel_ = 200;      //  deg/s
    // Max accelerations for smooth motion
    double maxLinearAcc_ = 3.0;          // m/s^2 (max permissible)
    double maxAngularAcc_ = 100;         // rad/s^2
};
