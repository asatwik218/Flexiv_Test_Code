#include "flexiv_tests/payload_test.h"
#include <flexiv/rdk/utility.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

#define _USE_MATH_DEFINES
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#pragma comment(lib, "winmm.lib")
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PayloadTest::PayloadTest(const std::string& robotSn)
    : FlexivRobotTest("PayloadTest", robotSn)
{}

inline double mmToM(double mm)
{
	return mm * 0.001;
}

inline double mToMM(double m) {
	return m * 1000;
}

inline double degToRad(double deg)
{
	constexpr double kPi = 3.14159265358979323846;
	return (deg * (kPi / 180.0)); // PI / 180.0
}

inline double radToDeg(double rad)
{
	constexpr double kPi = 3.14159265358979323846;
	return (rad / kPi * 180.0);
}

inline std::array<double, 4> eulerZYXToQuat(const std::array<double, 3>& euler)
{
	// euler = [roll (x), pitch (y), yaw (z)]
	Eigen::AngleAxisd rollAngle(euler[0], Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(euler[1], Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(euler[2], Eigen::Vector3d::UnitZ());

	// Compose rotations in ZYX order: R = yaw * pitch * roll
	Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

	// Return as {w, x, y, z}
	return { q.w(), q.x(), q.y(), q.z() };
}

inline std::array<double, 3> quatToEulerZYX(const std::array<double, 4>& quat)
{
	return flexiv::rdk::utility::Quat2EulerZYX(quat);
}


void PayloadTest::waitForJointMotionComplete(const std::vector<double>& targetPose, const std::string& phase)
{
    constexpr double pos_tolerance = 0.01;  // radians (~0.57 degrees)
    constexpr double vel_threshold = 0.01;  // rad/s
    auto wait_start = std::chrono::steady_clock::now();

    while (!stopRequested_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Check for faults
        if (robot_->fault()) {
            std::cerr << "[PayloadTest] FAULT DETECTED during " << phase << "!\n";
            throw std::runtime_error("Robot fault detected");
        }

        auto states = robot_->states();

        // Check if all joints reached target position and have low velocity
        bool all_joints_reached = true;
        for (size_t i = 0; i < 7; ++i) {
            double pos_error = std::abs(states.q[i] - targetPose[i]);
            double velocity = std::abs(states.dtheta[i]);

            if (pos_error >= pos_tolerance || velocity >= vel_threshold) {
                all_joints_reached = false;
                break;
            }
        }

        if (all_joints_reached) {
            std::cout << "[PayloadTest] Reached target position for " << phase << "\n";
            break;
        }

        // Timeout check (60 seconds for full pose)
        auto elapsed = std::chrono::steady_clock::now() - wait_start;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 60) {
            std::cerr << "[PayloadTest] Timeout waiting for " << phase << "\n";
            throw std::runtime_error("Motion timeout");
        }
    }
}

void PayloadTest::waitForCartesianMotionComplete(const std::vector<double>& targetPose, const std::string& phase)
{
    auto wait_start = std::chrono::steady_clock::now();

    while (!stopRequested_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Check for faults
        if (robot_->fault()) {
            std::cerr << "[PayloadTest] FAULT DETECTED during " << phase << "!\n";
            throw std::runtime_error("Robot fault detected");
        }

        auto states = robot_->states();

        // Check if TCP pose reached target (X, Y, Z, Rx, Ry, Rz)
        bool pose_reached = true;
        for (size_t i = 0; i < 6; ++i) {
            // Different tolerances for linear (0-2) vs angular (3-5) components
            double pos_tolerance = (i < 3) ? 0.005 : 0.05;  // 5mm for linear, ~3deg for angular
            double vel_threshold = (i < 3) ? 0.01 : 0.05;   // m/s for linear, rad/s for angular

            double pos_error = std::abs(states.tcp_pose[i] - targetPose[i]);
            double velocity = std::abs(states.tcp_vel[i]);

            if (pos_error >= pos_tolerance || velocity >= vel_threshold) {
                pose_reached = false;
                break;
            }
        }

        if (pose_reached) {
            std::cout << "[PayloadTest] Reached target position for " << phase << "\n";
            break;
        }

        // Timeout check (60 seconds for full pose)
        auto elapsed = std::chrono::steady_clock::now() - wait_start;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 60) {
            std::cerr << "[PayloadTest] Timeout waiting for " << phase << "\n";
            throw std::runtime_error("Motion timeout");
        }
    }
}

void PayloadTest::jointMotionTest(int testIndex, double payloadKg)
{
    try {
        std::cout << "[PayloadTest] Testing Joint Test " << (testIndex + 1)
                  << " with " << payloadKg << "kg payload\n";

        // Robot initialization
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_JOINT_POSITION);

        // Get test poses for this test (stored in DEGREES)
        const auto& initialPoseDeg = jointTestPoses_[testIndex].first;
        const auto& finalPoseDeg = jointTestPoses_[testIndex].second;

        // Convert from degrees to radians
        std::vector<double> initialPose(7);
        std::vector<double> finalPose(7);
        for (size_t i = 0; i < 7; ++i) {
            initialPose[i] = initialPoseDeg[i] * (M_PI / 180.0);
            finalPose[i] = finalPoseDeg[i] * (M_PI / 180.0);
        }

        std::cout << "[PayloadTest] Initial pose (deg): " << flexiv::rdk::utility::Vec2Str(initialPoseDeg, 3) << "\n";
        std::cout << "[PayloadTest] Final pose (deg): " << flexiv::rdk::utility::Vec2Str(finalPoseDeg, 3) << "\n";

        // Create velocity vectors (zero velocity for all joints)
        std::vector<double> velocities(7, 0.0);

        // Create max velocity and acceleration vectors (use 30% for initial positioning)
        std::vector<double> safe_max_velocities(7);
        std::vector<double> safe_max_accelerations(7);
        for (size_t i = 0; i < 7; ++i) {
            double max_vel_rad_s = maxJointVelocities_[i] * (M_PI / 180.0);
            double max_acc_rad_s2 = maxJointAccelerations_[i] * (M_PI / 180.0);
            safe_max_velocities[i] = max_vel_rad_s * 0.2;
            safe_max_accelerations[i] = max_acc_rad_s2 * 0.05;
        }

        // Move to initial position first (without logging) - use lower safe velocity
        std::cout << "[PayloadTest] Moving to initial position\n";
        robot_->SendJointPosition(initialPose, velocities, safe_max_velocities, safe_max_accelerations);

        // Wait for motion to complete
        waitForJointMotionComplete(initialPose, "motion to initial position");

        // Update state for logging
        {
            std::lock_guard<std::mutex> lk(mtx_);
            lastTestState_.currentTest = "JointTest" + std::to_string(testIndex + 1);
            lastTestState_.currentPayload = payloadKg;
        }

        // Create max velocity and acceleration vectors for test motion
        std::vector<double> test_max_velocities(7);
        std::vector<double> test_max_accelerations(7);
        for (size_t i = 0; i < 7; ++i) {
            double max_vel_rad_s = maxJointVelocities_[i] * (M_PI / 180.0);
            double max_acc_rad_s2 = maxJointAccelerations_[i] * (M_PI / 180.0);
            test_max_velocities[i] = max_vel_rad_s;
            test_max_accelerations[i] = max_acc_rad_s2;
        }

        // Now move to final position (logging will capture this motion)
        std::cout << "[PayloadTest] Moving to final position with max velocity\n";
        std::cout << "[PayloadTest] Max velocities (rad/s): " << flexiv::rdk::utility::Vec2Str(test_max_velocities, 3) << "\n";
        std::cout << "[PayloadTest] Max velocities (deg/s): ";
        for (size_t i = 0; i < 7; ++i) {
            std::cout << (test_max_velocities[i] * 180.0 / M_PI);
            if (i < 6) std::cout << ", ";
        }
        std::cout << "\n";
        std::cout << "[PayloadTest] Max accelerations (deg/s^2): ";
        for (size_t i = 0; i < 7; ++i) {
            std::cout << (test_max_accelerations[i] * 180.0 / M_PI);
            if (i < 6) std::cout << ", ";
        }
        std::cout << "\n";
        robot_->SendJointPosition(finalPose, velocities, test_max_velocities, test_max_accelerations);

        // Wait for motion to complete
        waitForJointMotionComplete(finalPose, "motion to final position");

        std::cout << "[PayloadTest] Completed Joint Test " << (testIndex + 1) << "\n";
    }
    catch (const std::exception& e) {
        std::cout << "Error in jointMotionTest: " << e.what() << std::endl;
        throw;
    }
}

void PayloadTest::cartesianMotionTest(int testIndex, double payloadKg)
{
    try {
        std::cout << "[PayloadTest] Testing Cartesian Test " << (testIndex + 1)
                  << " with " << payloadKg << "kg payload\n";

        // Robot initialization
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        // Get test poses for this test
        const auto& initialPose = cartesianTestPoses_[testIndex].first;
        const auto& finalPose = cartesianTestPoses_[testIndex].second;

        std::cout << "[PayloadTest] Initial TCP pose (X,Y,Z,Rx,Ry,Rz): ";
        for (size_t i = 0; i < 6; ++i) {
            std::cout << initialPose[i];
            if (i < 5) std::cout << ", ";
        }
        std::cout << "\n";

        std::cout << "[PayloadTest] Final TCP pose (X,Y,Z,Rx,Ry,Rz): ";
        for (size_t i = 0; i < 6; ++i) {
            std::cout << finalPose[i];
            if (i < 5) std::cout << ", ";
        }
        std::cout << "\n";

        // Extract position and orientation for initial pose
        std::array<double, 3> initial_position = {initialPose[0], initialPose[1], initialPose[2]};
        std::array<double, 3> initial_orientation = {
            initialPose[3] * (180.0 / M_PI),
            initialPose[4] * (180.0 / M_PI),
            initialPose[5] * (180.0 / M_PI)
        };

        // Extract position and orientation for final pose
        std::array<double, 3> final_position = {finalPose[0], finalPose[1], finalPose[2]};
        std::array<double, 3> final_orientation = {
            finalPose[3] * (180.0 / M_PI),
            finalPose[4] * (180.0 / M_PI),
            finalPose[5] * (180.0 / M_PI)
        };

        // Move to initial position first (without logging) - use lower safe velocity
        // Retry with reduced parameters if faults occur
        std::cout << "[PayloadTest] Moving to initial TCP pose\n";
        flexiv::rdk::Coord initial_target_pose(initial_position, initial_orientation, {"WORLD", "WORLD_ORIGIN"});

        double init_vel = 0.05;      // Start with 0.05 m/s
        double init_acc = 0.1;       // Start with 0.1 m/s^2
        double init_ang_vel = 50.0;  // Start with 50 deg/s
        bool initial_motion_complete = false;

        while (!initial_motion_complete && !stopRequested_) {
            std::cout << "[PayloadTest] Attempting initial move: vel=" << init_vel
                      << " m/s, acc=" << init_acc << " m/s^2, angVel=" << init_ang_vel << " deg/s\n";

            robot_->ExecutePrimitive("MoveL", {
                {"target", initial_target_pose},
                {"vel", init_vel},
                {"acc", init_acc},
                {"angVel", init_ang_vel},
                {"targetTolerLevel", 8}
            });

            // Wait for reached target and check for faults
            try {
                while (!std::get<int>(robot_->primitive_states()["reachedTarget"])) {
                    if (robot_->fault()) {
                        throw std::runtime_error("Robot fault during initial move");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                initial_motion_complete = true;
                std::cout << "[PayloadTest] Successfully reached initial position\n";
            }
            catch (const std::runtime_error& e) {
                std::cout << "[PayloadTest] Fault during initial move: " << e.what() << "\n";
                std::cout << "[PayloadTest] Clearing fault and retrying with reduced parameters...\n";
                robot_->ClearFault();
                std::this_thread::sleep_for(std::chrono::seconds(2));

                // Reduce parameters - can go as slow as needed
                init_vel *= 0.5;          // Halve velocity
                init_acc *= 0.5;          // Halve acceleration
                init_ang_vel *= 0.5;      // Halve angular velocity

                // Check if parameters are getting too small
                if (init_vel < 0.001 || init_acc < 0.001 || init_ang_vel < 1.0) {
                    std::cerr << "[PayloadTest] ERROR: Cannot reach initial position even with very slow motion!\n";
                    throw std::runtime_error("Failed to reach initial position");
                }
            }
        }

        // Update state for logging
        {
            std::lock_guard<std::mutex> lk(mtx_);
            lastTestState_.currentTest = "CartesianTest" + std::to_string(testIndex + 1);
            lastTestState_.currentPayload = payloadKg;
        }

        // Adaptive linear acceleration algorithm for linear motion tests
        double current_acceleration = maxLinearAcc_;     // Start at 3.0 m/s^2
        const double acc_reduction_step = 0.2;           // Reduce by 0.2 m/s^2
        const double min_acceleration = 0.2;             // Min linear acceleration

        bool motion_successful = false;
        int attempt = 1;

        while (!motion_successful && !stopRequested_) {
            // Check if we've reached minimum value
            if (current_acceleration < min_acceleration) break;

            std::cout << "[PayloadTest] Attempt " << attempt
                      << ": Testing with acceleration = " << current_acceleration << " m/s^2\n";

            // Create target pose
            flexiv::rdk::Coord final_target_pose(final_position, final_orientation, {"WORLD", "WORLD_ORIGIN"});

            // Execute MoveL - sweep linear acceleration, keep angular velocity constant
            robot_->ExecutePrimitive("MoveL", {
                {"target", final_target_pose},
                {"vel", maxTcpLinearVel_},           // max velocity
                {"acc", current_acceleration},       // current test acceleration
                {"angVel", maxTcpAngularVel_},       // constant angular velocity
                {"targetTolerLevel", 8}
            });

            // Monitor for faults during motion
            bool fault_occurred = false;
            try {
                // Wait for reached target
                while (!std::get<int>(robot_->primitive_states()["reachedTarget"])) {
                    // Check for faults during motion
                    if (robot_->fault()) {
                        throw std::runtime_error("Robot fault detected");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                motion_successful = true;  // If we reach here, no fault occurred
                std::cout << "[PayloadTest] SUCCESS! Motion completed without fault.\n";
            }
            catch (const std::runtime_error& e) {
                std::string error_msg(e.what());
                if (error_msg.find("fault") != std::string::npos) {
                    fault_occurred = true;
                    std::cout << "[PayloadTest] FAULT DETECTED: " << e.what() << "\n";
                } else {
                    // Other error (timeout, stop request, etc.) - rethrow
                    throw;
                }
            }

            if (fault_occurred) {
                std::cout << "[PayloadTest] Clearing fault...\n";
                robot_->ClearFault();
                std::this_thread::sleep_for(std::chrono::seconds(2));

                // Return to initial position before retrying - with fault handling
                std::cout << "[PayloadTest] Returning to initial position for retry...\n";
                double return_vel = 0.05;      // Start with safe values
                double return_acc = 0.5;
                double return_ang_vel = 50.0;
                bool return_successful = false;

                while (!return_successful && !stopRequested_) {
                    std::cout << "[PayloadTest] Attempting return: vel=" << return_vel
                              << " m/s, acc=" << return_acc << " m/s^2, angVel=" << return_ang_vel << " deg/s\n";

                    robot_->ExecutePrimitive("MoveL", {
                        {"target", initial_target_pose},
                        {"vel", return_vel},
                        {"acc", return_acc},
                        {"angVel", return_ang_vel},
                        {"targetTolerLevel", 8}
                    });

                    // Wait for reached target and check for faults
                    try {
                        while (!std::get<int>(robot_->primitive_states()["reachedTarget"])) {
                            if (robot_->fault()) {
                                throw std::runtime_error("Robot fault during return to initial");
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                        return_successful = true;
                        std::cout << "[PayloadTest] Successfully returned to initial position\n";
                    }
                    catch (const std::runtime_error& e) {
                        std::cout << "[PayloadTest] Fault during return: " << e.what() << "\n";
                        std::cout << "[PayloadTest] Clearing fault and retrying return with reduced parameters...\n";
                        robot_->ClearFault();
                        std::this_thread::sleep_for(std::chrono::seconds(2));

                        // Reduce parameters for return movement
                        return_vel *= 0.5;
                        return_acc *= 0.5;
                        return_ang_vel *= 0.5;

                        // Check if parameters are getting too small
                        if (return_vel < 0.001 || return_acc < 0.001 || return_ang_vel < 1.0) {
                            std::cerr << "[PayloadTest] ERROR: Cannot return to initial position even with very slow motion!\n";
                            throw std::runtime_error("Failed to return to initial position");
                        }
                    }
                }

                // Reduce acceleration by constant step
                current_acceleration -= acc_reduction_step;
                std::cout << "[PayloadTest] Reducing acceleration by " << acc_reduction_step
                          << " m/s^2. Next attempt will use " << current_acceleration << " m/s^2\n";
                attempt++;

                if (current_acceleration < min_acceleration) {
                    std::cerr << "[PayloadTest] ERROR: Could not complete motion even with minimum acceleration!\n";
                    throw std::runtime_error("Failed to find safe acceleration");
                }
            }
        }

        // Store the successful parameter
        if (motion_successful) {
            if (testIndex >= static_cast<int>(discoveredCartesianAccelerations_.size())) {
                discoveredCartesianAccelerations_.resize(testIndex + 1, 0.0);
            }
            discoveredCartesianAccelerations_[testIndex] = current_acceleration;
            std::cout << "[PayloadTest] Discovered maximum safe linear acceleration for Test "
                      << (testIndex + 1) << ": " << current_acceleration << " m/s^2\n";
        }

        std::cout << "[PayloadTest] Completed Cartesian Test " << (testIndex + 1) << "\n";
    }
    catch (const std::exception& e) {
        std::cout << "Error in cartesianMotionTest: " << e.what() << std::endl;
        throw;
    }
}

void PayloadTest::rotationalMotionTest(int testIndex, double payloadKg)
{
    try {
        std::cout << "[PayloadTest] Testing Rotational Test " << (testIndex + 1)
                  << " with " << payloadKg << "kg payload\n";

        // Get test poses for this rotational test
        const auto& initialPose = rotationalTestPoses_[testIndex].first;
        const auto& finalPose = rotationalTestPoses_[testIndex].second;

        // Convert Euler angles (degrees) to quaternions
        std::array<double, 4> initial_quat = eulerZYXToQuat({
            degToRad(initialPose[3]),
            degToRad(initialPose[4]),
            degToRad(initialPose[5])
        });

        std::array<double, 4> final_quat = eulerZYXToQuat({
            degToRad(finalPose[3]),
            degToRad(finalPose[4]),
            degToRad(finalPose[5])
        });

        // Create std::array<double, 7> for SendCartesianMotionForce
        // Format: {X, Y, Z, qw, qx, qy, qz}
        std::array<double, 7> initial_target_pose = {
            initialPose[0],      // X (already in meters)
            initialPose[1],      // Y
            initialPose[2],      // Z
            initial_quat[0],     // qw
            initial_quat[1],     // qx
            initial_quat[2],     // qy
            initial_quat[3]      // qz
        };

        std::array<double, 7> final_target_pose = {
            finalPose[0],        // X (already in meters)
            finalPose[1],        // Y
            finalPose[2],        // Z
            final_quat[0],       // qw
            final_quat[1],       // qx
            final_quat[2],       // qy
            final_quat[3]        // qz
        };

        // Zero force sensor before first SendCartesianMotionForce call
        std::cout << "[PayloadTest] Zeroing force sensor...\n";
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Switch to NRT_CARTESIAN_MOTION_FORCE mode for SendCartesianMotionForce
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});

        // Move to initial position first (without logging) - use lower safe velocity
        // Retry with reduced parameters if faults occur
        std::cout << "[PayloadTest] Moving to initial TCP pose\n";

        double init_vel = 0.05;           // Start with 0.05 m/s
        double init_lin_acc = 0.1;        // Start with 0.1 m/s^2
        double init_angular_vel = degToRad(50);  // 10 deg/s
        double init_ang_acc = degToRad(100);      // 50 deg/s^2
        bool initial_motion_complete = false;

        while (!initial_motion_complete && !stopRequested_) {
            std::cout << "[PayloadTest] Attempting initial move: vel=" << init_vel
                      << " m/s, lin_acc=" << init_lin_acc << " m/s^2, ang_vel=" << init_angular_vel
                      << " rad/s, ang_acc=" << init_ang_acc << " rad/s^2\n";

            try {
                // Send position command
                robot_->SendCartesianMotionForce(initial_target_pose, {}, {},
                                                 init_vel, init_lin_acc, init_angular_vel, init_ang_acc);

                // Wait 1 second before checking stopped()
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Loop and check robot_->stopped()
                while (!robot_->stopped() && !stopRequested_) {
                    // Check for faults
                    if (robot_->fault()) {
                        throw std::runtime_error("Robot fault during initial move");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                if (robot_->stopped()) {
                    initial_motion_complete = true;
                    std::cout << "[PayloadTest] Successfully reached initial position\n";
                }
            }
            catch (const std::runtime_error& e) {
                std::cout << "[PayloadTest] Fault during initial move: " << e.what() << "\n";
                std::cout << "[PayloadTest] Clearing fault and retrying with reduced parameters...\n";
                robot_->ClearFault();
                std::this_thread::sleep_for(std::chrono::seconds(2));

                // Reduce parameters - can go as slow as needed
                init_vel *= 0.5;
                init_lin_acc *= 0.5;
                init_angular_vel *= 0.5;
                init_ang_acc *= 0.5;

                // Check if parameters are getting too small
                if (init_vel < 0.001 || init_lin_acc < 0.001 || init_ang_acc < degToRad(1.0)) {
                    std::cerr << "[PayloadTest] ERROR: Cannot reach initial position even with very slow motion!\n";
                    throw std::runtime_error("Failed to reach initial position");
                }
            }
        }

        // Update state for logging
        {
            std::lock_guard<std::mutex> lk(mtx_);
            lastTestState_.currentTest = "RotationalTest" + std::to_string(testIndex + 1);
            lastTestState_.currentPayload = payloadKg;
        }

        // Adaptive parameter sweeping - start with user-specified values
        double current_linear_acc = 3.0;                    // 3.0 m/s^2
        double current_angular_acc = degToRad(500.0);       // 500 deg/s^2 converted to rad/s^2
        const double min_linear_acc = 0.1;                  // 0.1 m/s^2
        const double min_angular_acc = degToRad(2000);      // 10 deg/s^2

        bool motion_successful = false;
        int attempt = 1;

        while (!motion_successful && !stopRequested_) {
            // Check if we've reached minimum values
            if (current_linear_acc < min_linear_acc || current_angular_acc < min_angular_acc) {
                std::cerr << "[PayloadTest] ERROR: Reached minimum acceleration limits!\n";
                break;
            }

            std::cout << "[PayloadTest] Attempt " << attempt
                      << ": Testing with linear_acc=" << current_linear_acc << " m/s^2, "
                      << "angular_acc=" << current_angular_acc << " rad/s^2 ("
                      << current_angular_acc * (180.0 / M_PI) << " deg/s^2)\n";

            // Execute motion with SendCartesianMotionForce
            bool fault_occurred = false;
            try {
                // Send command to final position
                robot_->SendCartesianMotionForce(final_target_pose, {}, {},
                                                 maxTcpLinearVel_,                    // 1.0 m/s
                                                 current_linear_acc,                  // Current linear acc
                                                 maxTcpAngularVel_ * (M_PI / 180.0),  // 200 deg/s -> rad/s
                                                 current_angular_acc);                // Current angular acc

                // Wait 1 second before checking stopped()
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Loop and check robot_->stopped()
                while (!robot_->stopped() && !stopRequested_) {
                    // Check for faults
                    if (robot_->fault()) {
                        throw std::runtime_error("Robot fault detected during motion");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                if (robot_->stopped()) {
                    motion_successful = true;
                    std::cout << "[PayloadTest] SUCCESS! Motion completed without fault.\n";
                }
            }
            catch (const std::runtime_error& e) {
                fault_occurred = true;
                std::cout << "[PayloadTest] FAULT DETECTED: " << e.what() << "\n";
            }

            if (fault_occurred) {
                std::cout << "[PayloadTest] Clearing fault...\n";
                robot_->ClearFault();
                std::this_thread::sleep_for(std::chrono::seconds(2));
                robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);

                // Return to initial position before retrying - with fault handling
                std::cout << "[PayloadTest] Returning to initial position for retry...\n";
                double return_vel = 0.05;
                double return_lin_acc = 0.5;
                double return_ang_vel = degToRad(50);
                double return_ang_acc = degToRad(100);
                bool return_successful = false;

                while (!return_successful && !stopRequested_) {
                    std::cout << "[PayloadTest] Attempting return: vel=" << return_vel
                              << " m/s, lin_acc=" << return_lin_acc << " m/s^2, ang_acc="
                              << return_ang_acc << " rad/s^2\n";

                    try {
                        robot_->SendCartesianMotionForce(initial_target_pose, {}, {},
                                                         return_vel, return_lin_acc,
                                                         return_ang_vel, return_ang_acc);

                        // Wait 1 second before checking stopped()
                        std::this_thread::sleep_for(std::chrono::seconds(1));

                        // Loop and check robot_->stopped()
                        while (!robot_->stopped() && !stopRequested_) {
                            if (robot_->fault()) {
                                throw std::runtime_error("Robot fault during return to initial");
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }

                        if (robot_->stopped()) {
                            return_successful = true;
                            std::cout << "[PayloadTest] Successfully returned to initial position\n";
                        }
                    }
                    catch (const std::runtime_error& e) {
                        std::cout << "[PayloadTest] Fault during return: " << e.what() << "\n";
                        std::cout << "[PayloadTest] Clearing fault and retrying return with reduced parameters...\n";
                        robot_->ClearFault();
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);

                        // Reduce parameters for return movement
                        
                        return_lin_acc -= 0.5;
                        
                        return_ang_acc -= 1.5;

                        // Check if parameters are getting too small
                        if (return_vel < 0.001 || return_lin_acc < 0.001 || return_ang_acc < degToRad(1.0)) {
                            std::cerr << "[PayloadTest] ERROR: Cannot return to initial position even with very slow motion!\n";
                            throw std::runtime_error("Failed to return to initial position");
                        }
                    }
                }

                // Reduce both linear and angular acceleration for next attempt
                current_linear_acc *= 0.7;  // Reduce by 30%
                current_angular_acc *= 0.7; // Reduce by 30%

                std::cout << "[PayloadTest] Reducing accelerations for next attempt.\n";
                std::cout << "[PayloadTest] New linear_acc: " << current_linear_acc << " m/s^2\n";
                std::cout << "[PayloadTest] New angular_acc: " << current_angular_acc
                          << " rad/s^2 (" << current_angular_acc * (180.0 / M_PI) << " deg/s^2)\n";
                attempt++;
            }
        }

        // Store the successful parameters
        if (motion_successful) {
            if (testIndex >= static_cast<int>(discoveredAngularAccelerations_.size())) {
                discoveredAngularAccelerations_.resize(testIndex + 1, 0.0);
            }
            discoveredAngularAccelerations_[testIndex] = current_angular_acc;
            std::cout << "[PayloadTest] Discovered maximum safe angular acceleration for Test "
                      << (testIndex + 1) << ": " << current_angular_acc << " rad/s^2 ("
                      << current_angular_acc * (180.0 / M_PI) << " deg/s^2)\n";
            std::cout << "[PayloadTest] Corresponding linear acceleration: " << current_linear_acc << " m/s^2\n";
        }

        std::cout << "[PayloadTest] Completed Rotational Test " << (testIndex + 1) << "\n";
    }
    catch (const std::exception& e) {
        std::cout << "Error in rotationalMotionTest: " << e.what() << std::endl;
        throw;
    }
}

void PayloadTest::sequentialMotionTest(double payloadKg)
{
    try {
        std::cout << "[PayloadTest] Starting Sequential Motion Test with " << payloadKg << "kg payload\n";

        // Hardcoded sequence of 12 positions in mm and degrees (Euler angles)
        // Format: {X_mm, Y_mm, Z_mm, Rx_deg, Ry_deg, Rz_deg}
        std::vector<std::vector<double>> sequence_mm_deg = {
            { 690 , -113, 291 , 0.2 , 178, 0.2},
            {694, 337, 150, 0, 178, 0},
            {458, 267, 390, 0.86, 117, -1},
            {444, 276, 373, 86, 174, 7},
            {469, -3, 379, 87, 173, 7.70},
            {444, 276, 373, 86, 174, 7},
            {444, -28, 373, 86, 174, 7},
            {444, -28, 373, -1, 172, -5},
            {444, -28, 314, -70, 95, -69},
            {771, -28, 378, -70, 95, -69},
            {592, -28, 378, 0, -175, -5},
            {458, 267, 390, 0.86, 117, -1},
            {694, 337, 150, 0, 178, 0}
        };

        // Convert all positions from (mm, deg Euler) to (m, quaternion)
        // Format: std::array<double, 7> = {X, Y, Z, qw, qx, qy, qz}
        std::vector<std::array<double, 7>> sequence_poses;
        sequence_poses.reserve(sequence_mm_deg.size());

        for (const auto& pose_mm_deg : sequence_mm_deg) {
            // Convert mm to meters
            double x_m = mmToM(pose_mm_deg[0]);
            double y_m = mmToM(pose_mm_deg[1]);
            double z_m = mmToM(pose_mm_deg[2]);

            // Convert Euler angles (degrees) to quaternions
            std::array<double, 4> quat = eulerZYXToQuat({
                degToRad(pose_mm_deg[3]),
                degToRad(pose_mm_deg[4]),
                degToRad(pose_mm_deg[5])
            });

            // Create std::array<double, 7> pose
            std::array<double, 7> pose = {x_m, y_m, z_m, quat[0], quat[1], quat[2], quat[3]};
            sequence_poses.push_back(pose);
        }

        // Zero force sensor before first SendCartesianMotionForce call
        std::cout << "[PayloadTest] Zeroing force sensor...\n";
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Switch to NRT_CARTESIAN_MOTION_FORCE mode
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});

        // Update state for logging
        {
            std::lock_guard<std::mutex> lk(mtx_);
            lastTestState_.currentTest = "SequentialMotionTest";
            lastTestState_.currentPayload = payloadKg;
        }

        // Adaptive parameter sweeping - sweep BOTH angular velocity AND angular acceleration
        double current_linear_acc = 3.0;                    // 3.0 m/s^2
        double current_angular_vel = degToRad(200.0);       // 200 deg/s (max from header)
        double current_angular_acc = degToRad(500.0);       // 500 deg/s^2

        const double min_linear_acc = 0.1;                  // 0.1 m/s^2
        const double min_angular_vel = degToRad(10.0);      // 10 deg/s
        const double min_angular_acc = degToRad(10.0);      // 10 deg/s^2

        bool sequence_successful = false;
        int attempt = 1;

        while (!sequence_successful && !stopRequested_) {
            // Check if we've reached minimum values
            if (current_linear_acc < min_linear_acc ||
                current_angular_vel < min_angular_vel ||
                current_angular_acc < min_angular_acc) {
                std::cerr << "[PayloadTest] ERROR: Reached minimum parameter limits!\n";
                break;
            }

            std::cout << "[PayloadTest] Attempt " << attempt << ":\n";
            std::cout << "  Linear acc: " << current_linear_acc << " m/s^2\n";
            std::cout << "  Angular vel: " << current_angular_vel << " rad/s ("
                      << current_angular_vel * (180.0 / M_PI) << " deg/s)\n";
            std::cout << "  Angular acc: " << current_angular_acc << " rad/s^2 ("
                      << current_angular_acc * (180.0 / M_PI) << " deg/s^2)\n";

            // Execute the entire sequence
            bool fault_occurred = false;
            int waypoint_index = 0;

            try {
                for (waypoint_index = 0; waypoint_index < static_cast<int>(sequence_poses.size()); ++waypoint_index) {
                    if (stopRequested_) break;

                    std::cout << "[PayloadTest] Moving to waypoint " << (waypoint_index + 1)<< " / " << sequence_poses.size() << "\n";

                    const auto& target_pose = sequence_poses[waypoint_index];
                    
                    robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);

                    // Send command to target position
                    robot_->SendCartesianMotionForce(target_pose, {}, {},
                                                     maxTcpLinearVel_,      // 1.0 m/s
                                                     current_linear_acc,
                                                     current_angular_vel,
                                                     current_angular_acc);

                    // Wait 1 second before checking stopped()
                    std::this_thread::sleep_for(std::chrono::seconds(2));

                    // Loop and check robot_->stopped()
                    while (!robot_->stopped() && !stopRequested_) {
                        // Check for faults
                        if (robot_->fault()) {
                            throw std::runtime_error("Robot fault detected at waypoint "
                                                     + std::to_string(waypoint_index + 1));
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    if (robot_->stopped()) {
                        std::cout << "[PayloadTest] Reached waypoint " << (waypoint_index + 1) << "\n";
                    }
                }

                // If we completed all waypoints without fault
                if (waypoint_index == static_cast<int>(sequence_poses.size())) {
                    sequence_successful = true;
                    std::cout << "[PayloadTest] SUCCESS! Entire sequence completed without fault.\n";
                }
            }
            catch (const std::runtime_error& e) {
                fault_occurred = true;
                std::cout << "[PayloadTest] FAULT DETECTED: " << e.what() << "\n";
            }

            if (fault_occurred) {
                std::cout << "[PayloadTest] Clearing fault...\n";
                robot_->ClearFault();
                std::this_thread::sleep_for(std::chrono::seconds(2));
                robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);

                // Return to home position (first position in sequence)
                std::cout << "[PayloadTest] Returning to home position...\n";
                double return_vel = 0.05;
                double return_lin_acc = 0.5;
                double return_ang_vel = degToRad(10.0);
                double return_ang_acc = degToRad(50.0);
                bool return_successful = false;

                while (!return_successful && !stopRequested_) {
                    std::cout << "[PayloadTest] Attempting return to home...\n";
                    robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
                    try {
                        robot_->SendCartesianMotionForce(sequence_poses[0], {}, {},
                                                         return_vel, return_lin_acc,
                                                         return_ang_vel, return_ang_acc);

                        // Wait 1 second before checking stopped()
                        std::this_thread::sleep_for(std::chrono::seconds(1));

                        // Loop and check robot_->stopped()
                        while (!robot_->stopped() && !stopRequested_) {
                            if (robot_->fault()) {
                                throw std::runtime_error("Robot fault during return to home");
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }

                        if (robot_->stopped()) {
                            return_successful = true;
                            std::cout << "[PayloadTest] Successfully returned to home position\n";
                        }
                    }
                    catch (const std::runtime_error& e) {
                        std::cout << "[PayloadTest] Fault during return: " << e.what() << "\n";
                        std::cout << "[PayloadTest] Clearing fault and retrying return with reduced parameters...\n";
                        robot_->ClearFault();
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);


                        // Reduce parameters for return movement
                        return_vel *= 0.5;
                        return_lin_acc *= 0.5;
                        return_ang_vel *= 0.5;
                        return_ang_acc *= 0.5;

                        // Check if parameters are getting too small
                        if (return_vel < 0.001 || return_lin_acc < 0.001 || return_ang_acc < degToRad(1.0)) {
                            std::cerr << "[PayloadTest] ERROR: Cannot return to home position even with very slow motion!\n";
                            throw std::runtime_error("Failed to return to home position");
                        }
                    }
                }

                // Reduce all parameters for next attempt
                current_linear_acc *= 0.7;      // Reduce by 30%
                current_angular_vel *= 0.7;     // Reduce by 30%
                current_angular_acc *= 0.7;     // Reduce by 30%

                std::cout << "[PayloadTest] Reducing parameters for next attempt.\n";
                attempt++;
            }
        }

        // Store the successful parameters
        if (sequence_successful) {
            std::cout << "[PayloadTest] Discovered maximum safe parameters for sequential motion:\n";
            std::cout << "  Linear acceleration: " << current_linear_acc << " m/s^2\n";
            std::cout << "  Angular velocity: " << current_angular_vel << " rad/s ("
                      << current_angular_vel * (180.0 / M_PI) << " deg/s)\n";
            std::cout << "  Angular acceleration: " << current_angular_acc << " rad/s^2 ("
                      << current_angular_acc * (180.0 / M_PI) << " deg/s^2)\n";
        }

        std::cout << "[PayloadTest] Completed Sequential Motion Test\n";
    }
    catch (const std::exception& e) {
        std::cout << "Error in sequentialMotionTest: " << e.what() << std::endl;
        throw;
    }
}

void PayloadTest::log(std::ostream& out)
{
    // Read thread-safe state
    std::string testName{};
    double payload{0.0};
    {
        std::lock_guard<std::mutex> lk(mtx_);
        testName = lastTestState_.currentTest;
        payload = lastTestState_.currentPayload;
    }

    // Read robot state
    auto states = robot_->states();
    auto measuredJointPos = states.q;
    auto measuredJointVel = states.dq;
    auto measuredJointTorque = states.tau;
    auto measuredTcpPose = states.tcp_pose;
    auto measuredTcpVel = states.tcp_vel;

    // Write CSV row
    out << testName << ","
        << payload << ","
        << flexiv::rdk::utility::Vec2Str(measuredJointPos, 3, ",") << ","
        << flexiv::rdk::utility::Vec2Str(measuredJointVel, 3, ",") << ","
        << flexiv::rdk::utility::Vec2Str(measuredJointTorque, 3, ",") << ","
        << flexiv::rdk::utility::Arr2Str(measuredTcpPose, 3, ",") << ","
        << flexiv::rdk::utility::Arr2Str(measuredTcpVel, 3, ",");
}

void PayloadTest::performTest()
{
    #ifdef _WIN32
    // Improve timer resolution to 1ms on Windows
    timeBeginPeriod(1);
    #endif

    try {
        // Lambda to run a test phase with logging
        auto runPhase = [&](const std::string& logFilename, auto&& testFunc) -> bool {
            if (stopRequested_) return false;

            DataLogger::getInstance().setHeader("timestamp_ns,"
                "test_name,payload_kg,"
                "measured_q1,measured_q2,measured_q3,measured_q4,measured_q5,measured_q6,measured_q7,"
                "measured_dq1,measured_dq2,measured_dq3,measured_dq4,measured_dq5,measured_dq6,measured_dq7,"
                "measured_tau1,measured_tau2,measured_tau3,measured_tau4,measured_tau5,measured_tau6,measured_tau7,"
                "measured_x,measured_y,measured_z,measured_qw,measured_qx,measured_qy,measured_qz,"
                "measured_vx,measured_vy,measured_vz,measured_wx,measured_wy,measured_wz");
            startLogging(logFilename, 10);  // 10ms logging interval

            // Record initial pose
            auto init_states = robot_->states();
            std::ofstream pose_log("test_poses.txt", std::ios::app);
            pose_log << "\n[" << logFilename << "] Starting - ";
            pose_log << "Joints: " << flexiv::rdk::utility::Vec2Str(init_states.q, 3) << " | ";
            pose_log << "TCP: " << flexiv::rdk::utility::Arr2Str(init_states.tcp_pose, 3) << "\n";
            pose_log.close();

            bool fault_occurred = false;
            std::string fault_message;
            try {
                testFunc();
            }
            catch (const std::exception& e) {
                fault_occurred = true;
                fault_message = e.what();
                stopLogging();

                // Log fault
                std::ofstream fault_log("test_poses.txt", std::ios::app);
                fault_log << "[" << logFilename << "] FAULT: " << fault_message << "\n";
                fault_log.close();
                throw;
            }

            stopLogging();

            // Record final pose if no fault occurred
            if (!fault_occurred) {
                auto final_states = robot_->states();
                std::ofstream pose_log("test_poses.txt", std::ios::app);
                pose_log << "[" << logFilename << "] Completed - ";
                pose_log << "Joints: " << flexiv::rdk::utility::Vec2Str(final_states.q, 3) << " | ";
                pose_log << "TCP: " << flexiv::rdk::utility::Arr2Str(final_states.tcp_pose, 3) << "\n";
                pose_log.close();
            }

            std::this_thread::sleep_for(std::chrono::seconds(2));  // Rest between tests
            return !stopRequested_;
        };

        // Test each payload weight
        for (size_t i = 0; i < payloadWeights_.size(); ++i) {
            if (stopRequested_) break;

            double payload = payloadWeights_[i];
            std::cout << "\n===== Testing with " << payload << "kg payload =====\n";
            std::cout << "Please mount the " << payload << "kg payload and press Enter to continue...\n";
            std::cin.get();

            // // Test all joint test poses
            // for (size_t j = 0; j < 1; ++j) {
            //     if (stopRequested_) break;
            //     std::string filename = std::to_string((int)payload) + "kg_JointTest" +
            //                            std::to_string(j + 1) + ".csv";
            //     if (!runPhase(filename, [&]() { jointMotionTest(j, payload); })) return;
            // }

            // Test all Cartesian test poses (linear motion)
            // for (size_t c = 0; c < cartesianTestPoses_.size(); ++c) {
            //     if (stopRequested_) break;
            //     std::string filename = std::to_string((int)payload) + "kg_CartesianTest" +
            //                            std::to_string(c + 1) + ".csv";
            //     if (!runPhase(filename, [&]() { cartesianMotionTest(c, payload); })) return;
            // }

            // Test all rotational test poses (rotational motion)
            // for (size_t r = 0; r < rotationalTestPoses_.size(); ++r) {
            //     if (stopRequested_) break;
            //     std::string filename = std::to_string((int)payload) + "kg_RotationalTest" +
            //                            std::to_string(r + 1) + ".csv";
            //     if (!runPhase(filename, [&]() { rotationalMotionTest(r, payload); })) return;
            // }

            // Test sequential motion with hardcoded sequence
            if (!stopRequested_) {
                std::string filename = std::to_string((int)payload) + "kg_SequentialMotionTest.csv";
                if (!runPhase(filename, [&]() { sequentialMotionTest(payload); })) return;
            }

            // Pause after completing all tests for this payload (except for the last one)
            if (i < payloadWeights_.size() - 1 && !stopRequested_) {
                std::cout << "\n===== Completed all tests for " << payload << "kg payload =====\n";
                std::cout << "Please remove the " << payload << "kg payload and mount the "
                          << payloadWeights_[i + 1] << "kg payload.\n";
                std::cout << "Press Enter when ready to continue...\n";
                std::cin.get();
            }
        }

        std::cout << "\n[PayloadTest] All tests completed!\n";

        // Save discovered parameters to file
        if (!discoveredCartesianAccelerations_.empty() || !discoveredAngularAccelerations_.empty()) {
            std::ofstream results_log("discovered_parameters.txt");
            results_log << "===== Discovered Maximum Safe Parameters =====\n\n";

            // Linear motion tests (acceleration)
            if (!discoveredCartesianAccelerations_.empty()) {
                results_log << "Linear Motion Tests (Maximum Safe Linear Accelerations):\n";
                for (size_t i = 0; i < discoveredCartesianAccelerations_.size(); ++i) {
                    if (discoveredCartesianAccelerations_[i] > 0.0) {
                        results_log << "  Test " << (i + 1) << ": "
                                   << discoveredCartesianAccelerations_[i] << " m/s^2\n";
                    }
                }
                results_log << "\n";
            }

            // Rotational motion tests (angular acceleration)
            if (!discoveredAngularAccelerations_.empty()) {
                results_log << "Rotational Motion Tests (Maximum Safe Angular Accelerations):\n";
                for (size_t i = 0; i < discoveredAngularAccelerations_.size(); ++i) {
                    if (discoveredAngularAccelerations_[i] > 0.0) {
                        results_log << "  Test " << (i + 1) << ": "
                                   << discoveredAngularAccelerations_[i] << " rad/s^2\n";
                    }
                }
            }

            results_log.close();
            std::cout << "[PayloadTest] Saved discovered parameters to 'discovered_parameters.txt'\n";
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[PayloadTest] performTest error: " << e.what() << "\n";
    }

    #ifdef _WIN32
    timeEndPeriod(1);
    #endif
}
