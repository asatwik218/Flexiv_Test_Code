#include "flexiv_tests/response_time_test.h"
#include <flexiv/rdk/utility.hpp>
#include <iostream>
#include <chrono>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#pragma comment(lib, "winmm.lib")
#endif

ResponseTimeTest::ResponseTimeTest(const std::string& robotSn)
    : FlexivRobotTest("ResponseTimeTest", robotSn)
{}

void ResponseTimeTest::executeTest()
{
    
    try {
        // Empty wrench and velocity arrays (position-only control)
        std::array<double, 6> zeroWrench = {};
        std::array<double, 6> zeroVelocity = {};

        // Timing setup
        auto commandStartTime = std::chrono::steady_clock::now();
        auto nextTick = commandStartTime;

        std::cout << "[ResponseTimeTest] Sending " << testPositions_.size()
                  << " commands with individual delays\n";

        // Send commands with their individual delay intervals
        for (size_t i = 0; i < testPositions_.size() && !stopRequested_; ++i) {
            const auto& [targetPose, delayMs] = testPositions_[i];

            // Wait for the specified delay BEFORE sending this command
            if (delayMs > 0) {
                nextTick += std::chrono::milliseconds(delayMs);
                std::this_thread::sleep_until(nextTick);
            }

            // std::cout << "[ResponseTimeTest] Command " << i << "/" << testPositions_.size()
            //           << " - Delay before: " << delayMs << "ms - Target: ["
            //           << targetPose[0] << ", " << targetPose[1] << ", " << targetPose[2] << "]\n";

            // Send command with high velocity and maximum acceleration
            robot_->SendCartesianMotionForce(
                targetPose,
                zeroWrench,
                zeroVelocity,
                maxLinearVel_,
                maxAngularVel_,
                maxLinearAcc_,
                maxAngularAcc_
            );
        }

        std::cout << "[ResponseTimeTest] All commands sent. Waiting "
                  << settlingTimeMs_ << "ms for robot to settle...\n";

        // Wait for robot to complete motion (check stopRequested periodically)
        auto settleStart = std::chrono::steady_clock::now();
        while (!stopRequested_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - settleStart
            ).count();
            if (elapsed >= settlingTimeMs_) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception& e) {
        std::cout << "[ResponseTimeTest] Error in executeDelayTest: " << e.what() << std::endl;
        throw;
    }
}

void ResponseTimeTest::log(std::ostream& out)
{
    // Read robot state (robot_->states() is thread-safe per RDK design)
    auto states = robot_->states();
    auto tcpPose = states.tcp_pose;
    auto tcpVel = states.tcp_vel;

    out << flexiv::rdk::utility::Arr2Str(tcpPose, 7, ",") << ","
        << flexiv::rdk::utility::Arr2Str(tcpVel, 6, ",");
}

void ResponseTimeTest::performTest()
{
    #ifdef _WIN32
    // Improve timer resolution to 1ms on Windows
    timeBeginPeriod(1);
    #endif

    try {
        // Robot initialization - do once before all tests
        std::cout << "[ResponseTimeTest] Initializing robot...\n";
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        std::cout << "[ResponseTimeTest] Zeroing FT sensor, waiting 2 seconds...\n";

        // Wait 2 seconds (check stopRequested periodically)
        for (int i = 0; i < 20 && !stopRequested_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (stopRequested_) return;
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});
        std::cout << "[ResponseTimeTest] Robot initialized and ready!\n\n";

        // Move to starting pose and wait for arrival
        std::cout << "[ResponseTimeTest] Moving to starting pose...\n";
        std::array<double, 7> startingPose = {0.684, -0.595, 0.128, 0, 0, -1, 0};
        std::array<double, 6> zeroWrench = {0, 0, 0, 0, 0, 0};
        std::array<double, 6> zeroVel = {0, 0, 0, 0, 0, 0};

        // Send starting pose command
        robot_->SendCartesianMotionForce(
            startingPose,
            zeroWrench,
            zeroVel,
            maxLinearVel_,
            maxAngularVel_,
            maxLinearAcc_,
            maxAngularAcc_
        );

        // Wait for robot to reach starting pose (check position error)
        bool reached = false;
        auto startTime = std::chrono::steady_clock::now();
        while (!reached && !stopRequested_) {
            auto currentPose = robot_->states().tcp_pose;
            double posError = std::sqrt(
                std::pow(startingPose[0] - currentPose[0], 2) +
                std::pow(startingPose[1] - currentPose[1], 2) +
                std::pow(startingPose[2] - currentPose[2], 2)
            );

            if (posError < 0.005) {  // 5mm tolerance
                reached = true;
                std::cout << "[ResponseTimeTest] Starting pose reached (error: " << posError*1000 << " mm)\n";
            }

            // Timeout after 30 seconds
            auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
            if (elapsed > 30.0) {
                std::cout << "[ResponseTimeTest] Timeout waiting for starting pose (error: " << posError*1000 << " mm)\n";
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Brief pause before test (check stopRequested)
        for (int i = 0; i < 10 && !stopRequested_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (stopRequested_) return;

        std::cout << "[ResponseTimeTest] Ready to start test!\n\n";

        // Configure logging header
        DataLogger::getInstance().setHeader(
            "timestamp_ns,"
            "tcp_x,tcp_y,tcp_z,tcp_qw,tcp_qx,tcp_qy,tcp_qz,"
            "tcp_vx,tcp_vy,tcp_vz,tcp_wx,tcp_wy,tcp_wz"
        );

        std::string filename = "response_time_test.csv";

        std::cout << "[ResponseTimeTest] ======================================\n";
        std::cout << "[ResponseTimeTest] Starting Response Time Test\n";
        std::cout << "[ResponseTimeTest] ======================================\n";
        std::cout << "[ResponseTimeTest] Starting logging: " << filename << "\n";
        startLogging(filename, 1);  // 1ms logging interval

        try {
            executeTest();
        }
        catch (...) {
            stopLogging();
            throw;
        }

        stopLogging();
        std::cout << "[ResponseTimeTest] Logging stopped.\n";

        std::cout << "[ResponseTimeTest] All tests completed successfully!\n";
    }
    catch (const std::exception& e) {
        std::cerr << "[ResponseTimeTest] performTest error: " << e.what() << "\n";
    }

    #ifdef _WIN32
    timeEndPeriod(1);
    #endif
}
