#include "flexiv_tests/position_streaming_test.h"
#include <flexiv/rdk/utility.hpp>
#include <iostream>
#include <chrono>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#pragma comment(lib, "winmm.lib")
#endif

PositionStreamingTest::PositionStreamingTest(const std::string& robotSn,
                                             const std::array<double, 6>& startingPose)
    : FlexivRobotTest("PositionStreamingTest", robotSn),
      startingPose_mm_deg_(startingPose)
{}

void PositionStreamingTest::moveToStartingPose()
{
    std::cout << "[PositionStreamingTest] Moving to starting pose: x=" << startingPose_mm_deg_[0]
              << " mm, y=" << startingPose_mm_deg_[1] << " mm, z=" << startingPose_mm_deg_[2]
              << " mm, rx=" << startingPose_mm_deg_[3] << " deg, ry=" << startingPose_mm_deg_[4]
              << " deg, rz=" << startingPose_mm_deg_[5] << " deg\n";

    robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
    robot_->SetForceControlAxis({false, false, false, false, false, false});

    auto target_pose_m_quat = convertPose_mmDeg_to_mQuat(startingPose_mm_deg_);
    robot_->SendCartesianMotionForce(target_pose_m_quat, {}, {}, 0.08);  // 0.1 m/s velocity

    // Wait until starting pose is reached
    while (!isCartesianPoseReached(startingPose_mm_deg_, {true, true, true, false, false, false})) {
        if (stopRequested_) {
            std::cout << "[PositionStreamingTest] Stop requested during move to starting pose\n";
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    robot_->Stop();
    std::cout << "[PositionStreamingTest] Starting pose reached\n";
}

void PositionStreamingTest::constantVelocityTest(double velocity, uint16_t durationSeconds, uint16_t streamIntervalMs, bool useCurrentPose)
{
    try {
        std::cout << "[constant velocity test] Starting \n";

        // Robot initialization
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});

        // Get initial pose and setup
        auto target_pose = robot_->states().tcp_pose;
        const double dt = streamIntervalMs / 1000.0;

        // Timing
        auto loop_start = std::chrono::steady_clock::now();
        auto next_tick = loop_start;

        while (!stopRequested_) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - loop_start).count();
            if (elapsed >= durationSeconds) break;

            next_tick += std::chrono::milliseconds(streamIntervalMs);

            if(useCurrentPose){
                auto curr_pose = robot_->states().tcp_pose;
                target_pose[0] = curr_pose[0] + velocity * dt; //increment from current curr_pose
            }
            else{
                target_pose[0] += velocity * dt ; //increment from previous target pose
            }

            // Send command with zero velocity and high motion limits
            robot_->SendCartesianMotionForce(target_pose, {}, {}, MAX_LINEAR_VEL, MAX_ANGULAR_VEL, MAX_LINEAR_ACC, MAX_ANGULAR_ACC);

            // // Update commanded values for logging
            // setCommandedTcpPose(target_pose);
            // setCommandedTcpVel({velocity, 0, 0, 0, 0, 0});

            std::this_thread::sleep_until(next_tick);
        }


        std::cout << "[constant velocity test] Completed\n";
    }
    catch (const std::exception& e) {
        std::cout << "Error in constantVelocityTest: " << e.what() << std::endl;
        throw;
    }
}

void PositionStreamingTest::varyingVelocityTest(double minVelocity, double maxVelocity, uint16_t durationSeconds, uint16_t streamIntervalMs, bool useCurrentPose)
{
    try {
        std::cout << "[varying velocity curve test] Starting \n";

        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});

        // Get initial pose and setup
        auto target_pose = robot_->states().tcp_pose;
        const double dt = streamIntervalMs / 1000.0;

        const double total_duration = static_cast<double>(durationSeconds);
        const double velocity_range = maxVelocity - minVelocity;

        auto loop_start = std::chrono::steady_clock::now();
        auto next_tick = loop_start;

        while (!stopRequested_) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - loop_start).count();
            if (elapsed >= total_duration) break;

            next_tick += std::chrono::milliseconds(streamIntervalMs);

            // Linear interpolation of velocity
            double progress = elapsed / total_duration;  // 0.0 to 1.0
            double current_velocity = minVelocity + velocity_range * progress;

            // Integrate from measured or target pose
            if(useCurrentPose){
                auto measured_pose = robot_->states().tcp_pose;
                target_pose[0] = measured_pose[0] + current_velocity * dt;
            }
            else{
                target_pose[0] += current_velocity * dt;
            }

            robot_->SendCartesianMotionForce(target_pose, {}, {}, MAX_LINEAR_VEL, MAX_ANGULAR_VEL, MAX_LINEAR_ACC, MAX_ANGULAR_ACC);

            // // Update commanded values for logging
            // setCommandedTcpPose(target_pose);
            // setCommandedTcpVel({current_velocity, 0, 0, 0, 0, 0});

            std::this_thread::sleep_until(next_tick);
        }

        std::cout << "[varying velocity test] Completed\n";
    }
    catch (const std::exception& e) {
        std::cout << "Error in varyingVelocityTest: " << e.what() << std::endl;
        throw;
    }
}

void PositionStreamingTest::periodicCurveTest(
    double xAmplitude,
    double zAmplitude,
    double frequency,
    uint16_t numCycles,
    uint16_t streamIntervalMs)
{
    try {
        std::cout << "[periodic curve test] Starting\n";

        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});

        // Initial pose = center of oscillation
        const auto init_pose = robot_->states().tcp_pose;
        auto target_pose = init_pose;

        constexpr double TWO_PI = 6.283185307179586;
        const double omega = TWO_PI * frequency;
        const double period = 1.0 / frequency;

        const auto dt = std::chrono::milliseconds(streamIntervalMs);

        for (uint16_t cycle = 0; cycle < numCycles && !stopRequested_; ++cycle) {

            auto cycle_start = std::chrono::steady_clock::now();
            auto next_tick = cycle_start;

            while (!stopRequested_) {
                auto now = std::chrono::steady_clock::now();
                double t = std::chrono::duration<double>(now - cycle_start).count();

                if (t >= period)
                    break;

                // Analytic sinusoidal position (NO integration)
                target_pose[0] = init_pose[0] + xAmplitude * std::sin(omega * t);
                target_pose[2] = init_pose[2] + zAmplitude * std::sin(omega * t);

                robot_->SendCartesianMotionForce(
                    target_pose,
                    {}, {},
                    MAX_LINEAR_VEL,
                    MAX_ANGULAR_VEL,
                    MAX_LINEAR_ACC,
                    MAX_ANGULAR_ACC);

                next_tick += dt;
                std::this_thread::sleep_until(next_tick);
            }

            // Ensure next cycle starts cleanly
            target_pose = init_pose;
        }

        
        std::cout << "[periodic curve test] Completed\n";
    }
    catch (const std::exception& e) {
        std::cout << "Error in periodicCurveTest: " << e.what() << std::endl;
        throw;
    }
}

void PositionStreamingTest::stepPositionTest(double positionA_offset_m, double positionB_offset_m,
                                             double moveVelocity, uint16_t holdDuration_s,
                                             uint16_t numCycles, uint16_t streamIntervalMs, bool useCurrentPose)
{
    try {
        std::cout << "[StepPositionTest] Starting test with " << numCycles << " cycles\n";
        std::cout << "[StepPositionTest] Position A offset: " << positionA_offset_m << " m, Position B offset: " << positionB_offset_m << " m\n";
        std::cout << "[StepPositionTest] Move velocity: " << moveVelocity << " m/s, Hold duration: " << holdDuration_s << " s\n";
        std::cout << "[StepPositionTest] Stream interval: " << streamIntervalMs << " ms\n";

        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});

        // Get initial pose as reference
        auto init_pose = robot_->states().tcp_pose;

        // Define position A and B (offset in X direction from initial pose)
        auto positionA = init_pose;
        positionA[0] += positionA_offset_m;

        auto positionB = init_pose;
        // positionB[0] += positionB_offset_m;

        auto next_tick = std::chrono::steady_clock::now();
        const double dt = streamIntervalMs / 1000.0;

        // Loop through cycles: A -> B -> A -> B ...
        for (uint16_t cycle = 0; cycle < numCycles && !stopRequested_; ++cycle) {
            // std::cout << "[StepPositionTest] Cycle " << (cycle + 1) << "/" << numCycles << "\n";

            // ========== PHASE 1: Move to Position A and Hold =========

            auto target_pose = robot_->states().tcp_pose;
            auto phase_start = std::chrono::steady_clock::now();

            // Move to position A with streaming
            while (!stopRequested_) {
                auto measured_pose = robot_->states().tcp_pose;

                next_tick += std::chrono::milliseconds(streamIntervalMs);

                if( abs(target_pose[0] - positionA[0]) > 0.005 ){
                    if(useCurrentPose){
                        target_pose[0] = measured_pose[0] + moveVelocity * dt;
                    }
                    else{
                        target_pose[0] += moveVelocity * dt;
                    }
                }
                else{
                    break;
                }
                robot_->SendCartesianMotionForce(target_pose, {}, {}, MAX_LINEAR_VEL, MAX_ANGULAR_VEL, MAX_LINEAR_ACC, MAX_ANGULAR_ACC);

                setCommandedTcpPose(target_pose);
                setCommandedTcpVel({moveVelocity, 0, 0, 0, 0, 0});

                std::this_thread::sleep_until(next_tick);
            }

            // Hold at position A
            phase_start = std::chrono::steady_clock::now();
            while (!stopRequested_) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - phase_start).count();
                if (elapsed >= holdDuration_s) break;

                next_tick += std::chrono::milliseconds(streamIntervalMs);

                // Stream position A repeatedly
                robot_->SendCartesianMotionForce(positionA, {}, {}, MAX_LINEAR_VEL, MAX_ANGULAR_VEL, MAX_LINEAR_ACC, MAX_ANGULAR_ACC);

                setCommandedTcpPose(positionA);
                setCommandedTcpVel({0, 0, 0, 0, 0, 0});

                std::this_thread::sleep_until(next_tick);
            }

            // ========== PHASE 2: Move to Position B and Hold ==========
            // std::cout << "[StepPositionTest] Moving to Position B (offset: " << positionB_offset_m << " m)\n";

            target_pose = robot_->states().tcp_pose;
            moveVelocity *= -1;

            // Move to position B with streaming
            while (!stopRequested_) {
                auto measured_pose = robot_->states().tcp_pose;

                next_tick += std::chrono::milliseconds(streamIntervalMs);
                
                if( abs(target_pose[0] - positionB[0]) > 0.005 )
                    if(useCurrentPose)
                       target_pose[0] = measured_pose[0] + moveVelocity * dt;
                    else
                        target_pose[0] += moveVelocity * dt;
                else
                    break;

                robot_->SendCartesianMotionForce(target_pose, {}, {}, MAX_LINEAR_VEL, MAX_ANGULAR_VEL, MAX_LINEAR_ACC, MAX_ANGULAR_ACC);

                setCommandedTcpPose(target_pose);
                setCommandedTcpVel({moveVelocity, 0, 0, 0, 0, 0});

                std::this_thread::sleep_until(next_tick);
            }

            // Hold at position B
            phase_start = std::chrono::steady_clock::now();
            while (!stopRequested_) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - phase_start).count();
                if (elapsed >= holdDuration_s) break;

                next_tick += std::chrono::milliseconds(streamIntervalMs);

                // Stream position B repeatedly
                robot_->SendCartesianMotionForce(positionB, {}, {}, MAX_LINEAR_VEL, MAX_ANGULAR_VEL, MAX_LINEAR_ACC, MAX_ANGULAR_ACC);

                setCommandedTcpPose(positionB);
                setCommandedTcpVel({0, 0, 0, 0, 0, 0});

                std::this_thread::sleep_until(next_tick);
            }

            moveVelocity *= -1;
        }
        std::cout << "[StepPositionTest] Completed all cycles. Analyze step response in logged data.\n";
    }
    catch (const std::exception& e) {
        std::cout << "Error in stepPositionTest: " << e.what() << std::endl;
        throw;
    }
}

void PositionStreamingTest::stepVelocityTest(double lowVelocity, double highVelocity, double velocityStepSize,
                                              uint16_t durationPerStep, uint16_t streamIntervalMs, bool useCurrentPose)
{
    try {
        std::cout << "[stepVelocityTest] Starting step velocity test\n";
        std::cout << "[stepVelocityTest] Velocity range: " << lowVelocity << " to " << highVelocity << " m/s\n";
        std::cout << "[stepVelocityTest] Step size: " << velocityStepSize << " m/s\n";
        std::cout << "[stepVelocityTest] Duration per step: " << durationPerStep << " s\n";
        std::cout << "[stepVelocityTest] Stream interval: " << streamIntervalMs << " ms\n";

        // Robot initialization
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});

        // Get initial pose
        auto target_pose = robot_->states().tcp_pose;
        const double dt = streamIntervalMs / 1000.0;

        // Calculate number of steps
        int numSteps = static_cast<int>(std::ceil((highVelocity - lowVelocity) / velocityStepSize)) + 1;
        std::cout << "[stepVelocityTest] Number of velocity steps: " << numSteps << "\n";

        // Loop through each velocity step
        for (int step = 0; step < numSteps && !stopRequested_; ++step) {
            double currentVelocity = lowVelocity + (step * velocityStepSize);

            // Clamp to high velocity
            if (currentVelocity > highVelocity) {
                currentVelocity = highVelocity;
            }

            std::cout << "[stepVelocityTest] Step " << (step + 1) << "/" << numSteps
                      << ": Streaming velocity = " << currentVelocity << " m/s for "
                      << durationPerStep << " s\n";

            // Timing for this step
            auto step_start = std::chrono::steady_clock::now();
            auto next_tick = step_start;

            // Stream at constant velocity for durationPerStep seconds
            while (!stopRequested_) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - step_start).count();

                // Break when duration elapsed
                if (elapsed >= durationPerStep) break;

                next_tick += std::chrono::milliseconds(streamIntervalMs);

                // Integrate position based on current or target pose
                if (useCurrentPose) {
                    auto curr_pose = robot_->states().tcp_pose;
                    target_pose[0] = curr_pose[0] + currentVelocity * dt;
                }
                else {
                    target_pose[0] += currentVelocity * dt;
                }

                // Send command with high motion limits
                robot_->SendCartesianMotionForce(target_pose, {}, {},
                    MAX_LINEAR_VEL, MAX_ANGULAR_VEL, MAX_LINEAR_ACC, MAX_ANGULAR_ACC);

                // Update commanded values for logging
                setCommandedTcpPose(target_pose);
                setCommandedTcpVel({currentVelocity, 0, 0, 0, 0, 0});

                std::this_thread::sleep_until(next_tick);
            }

            if (stopRequested_) break;
        }

        std::cout << "[stepVelocityTest] Completed\n";
    }
    catch (const std::exception& e) {
        std::cout << "Error in stepVelocityTest: " << e.what() << std::endl;
        throw;
    }
}

void PositionStreamingTest::performTest()
{
    #ifdef _WIN32
    // Improve timer resolution to 1ms on Windows
    timeBeginPeriod(1);
    #endif

    try {
        // Define fields to log: commanded pose, commanded vel, actual pose, actual vel
        std::vector<LogField> fieldsToLog = {
            LogField::CMD_TCP_POSE,
            LogField::CMD_TCP_VEL,
            LogField::TCP_POSE,
            LogField::TCP_VEL
        };

        auto runPhase = [&](const std::string& logFilename, auto&& testFunc) -> bool {
            if (stopRequested_) return false;
            
            //ZeroFTSensor
            ZeroFTSensor();

            // Move to starting pose before each test
            moveToStartingPose();
            if (stopRequested_) return false;

            std::this_thread::sleep_for(std::chrono::seconds(3));

            startLogging(logFilename, fieldsToLog, 1);  // 1ms logging interval

            try {
                testFunc();
            }
            catch (...) {
                stopLogging();
                throw;
            }

            stopLogging();

            std::this_thread::sleep_for(std::chrono::seconds(5));
            return !stopRequested_;
        };

        // Test intervals
        std::vector<uint16_t> intervals = {1, 10 , 25, 50 , 75 , 100};
        for (auto interval : intervals) {

            // // Constant velocity test (0.10 m/s)
            // std::string filename0 = "prev_constant_velocity_v0_10_interval_" + std::to_string(interval) + "ms.csv";
            // if (!runPhase(filename0, [&]() { constantVelocityTest(0.10, 10, interval, false); })) return;

            // // High-velocity varying test (0.05 to 0.30 m/s, higher acceleration)
            // std::string filename1 = "prev_target_varying_v0_05-0_30_interval_" + std::to_string(interval) + "ms.csv";
            // if (!runPhase(filename1, [&]() { varyingVelocityTest(0.05, 0.30, 5, interval, false); })) return;

            // // Periodic test with higher frequency (1.0 Hz = 1s period)
            // std::string filename2 = "prev_target_periodic_xamp0_05_zamp0_05_freq1_0Hz_interval_" + std::to_string(interval) + "ms.csv";
            // if (!runPhase(filename2, [&]() { periodicCurveTest(0.05, 0.05, 1, 5, interval); })) return;
        

            // // Step position test: Move between +0.2m and -0.2m, hold 5s each, 3 cycles
            // std::string filename3 = "prev_step_position_test_interval_" + std::to_string(interval) + "ms.csv";
            // if (!runPhase(filename3, [&]() { stepPositionTest(0.4, -0.4, 0.1, 10, 3, interval, false); })) return;
            
            // // Constant velocity test (0.10 m/s)
            // std::string filename4 = "curr_constant_velocity_v0_10_interval_" + std::to_string(interval) + "ms.csv";
            // if (!runPhase(filename4, [&]() { constantVelocityTest(0.10, 10, interval, true); })) return;

            // // High-velocity varying test (0.05 to 0.30 m/s, higher acceleration)
            // std::string filename5 = "curr_target_varying_v0_05-0_30_interval_" + std::to_string(interval) + "ms.csv";
            // if (!runPhase(filename5, [&]() { varyingVelocityTest(0.05, 0.30, 10, interval, true); })) return;

            //Step position test: Move between +0.2m and -0.2m, hold 5s each, 3 cycles
            // if(interval >= 10){
            //     std::string filename7 = "curr_step_position_test_interval_" + std::to_string(interval) + "ms.csv";
            //     if (!runPhase(filename7, [&]() { stepPositionTest(0.5, -0.5, 0.1, 5, 3, interval, true); })) return;
            // }

            // Step velocity test: Start at 0.05 m/s, step up by 0.1 m/s every 5s until 0.30 m/s
        
            std::string filename8 = "prev_step_velocity_test_interval_" + std::to_string(interval) + "ms.csv";
            if (!runPhase(filename8, [&]() { stepVelocityTest(0.1, 0.30, 0.1, 2, interval, false); })) return;
        
        }

    }
    catch (const std::exception& e) {
        std::cerr << "[PositionStreamingTest] performTest error: " << e.what() << "\n";
    }

    #ifdef _WIN32
    timeEndPeriod(1);
    #endif
}
