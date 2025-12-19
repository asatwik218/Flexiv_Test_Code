#pragma once

#include "flexiv_tests/flexiv_robot_test.h"
#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>
#include <limits>

class AdmittanceTest : public FlexivRobotTest
{
public:
    // Constructor: robot serial number, admittance enabled, test duration (s),
    // starting/end poses in mm/deg (x,y,z,rx,ry,rz), scalar pos_m (kg), pos_k (N/m),
    // high_end (max force per axis) in N (size 3)
    AdmittanceTest(
        const std::string& robotSn,
        bool isAdmittance,
        uint16_t testDuration,
        const std::array<double, 6>& startingPose_mm_deg,
        const std::array<double, 6>& endPose_mm_deg,
        double pos_m,
        double pos_k,
        const std::array<double, 3>& high_end);

    ~AdmittanceTest() override = default;

    // Move robot to starting pose (uses Flexiv API helpers from base)
    void moveToStartingPose();

    // Main admittance control loop (streamIntervalMs in ms)
    void admittanceControlLoop(uint16_t streamIntervalMs);

    // High-level test runner
    void performTest();

private:
    // Helper static inline functions (Eigen-based)
    static inline Eigen::Vector3d applySmoothDeadZone3(
        const Eigen::Vector3d& input_s,
        const Eigen::Vector3d& low_end,
        const Eigen::Vector3d& high_end,
        double sw_gn = 100.0);

    static inline Eigen::Vector3d applySmoothSaturation(
        const Eigen::Vector3d& input,
        double max_limit);

    Eigen::Vector3d generateMotionDynamics(
        const Eigen::Vector3d& cur_position,
        const Eigen::Vector3d& cur_lin_vel_ee,
        const Eigen::Vector3d& adm_force_ee,
        double dt) const;

private:
    // Member variables (m_ prefix)
    std::array<double, 6> m_startingPose_mm_deg;
    std::array<double, 6> m_endPose_mm_deg;

    bool m_isAdmittance = false;
    uint16_t m_testDuration = 0; // seconds

    // Scalar gains (virtual mass, stiffness, damping)
    double m_pos_m = 1.0;
    double m_pos_k = 50.0;
    double m_pos_d = 0.0;

    // Gain matrices (direct members instead of map for better performance)
    Eigen::Matrix3d m_K;      // Stiffness matrix
    Eigen::Matrix3d m_D;      // Damping matrix
    Eigen::Matrix3d m_InvM;   // Inverse mass matrix

    // Admittance force thresholds (Eigen vectors)
    Eigen::Vector3d m_high_end = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_low_end  = Eigen::Vector3d::Zero();

    // Desired/equilibrium position (m) and current velocity (m/s)
    Eigen::Vector3d m_desiredPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_currentVelocity = Eigen::Vector3d::Zero();

    // Runtime parameters (tune as needed)
    double m_switchingGain = 100.0;
    double m_maxLinearVel = 1.0;   // [m/s] safe default
    double m_maxAngularVel = 10.0;  // [rad/s]
    double m_maxLinearAcc = 10.0;   // [m/s^2]
    double m_maxAngularAcc = 100.0;  // [rad/s^2]
};

