#include "flexiv_tests/velocity_tracking_test.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <flexiv/rdk/utility.hpp>

#ifdef _WIN32
    #include <Windows.h>
#endif

VelocityTrackingTest::VelocityTrackingTest(const std::string& robotSn):FlexivRobotTest("VelocityTrackingTest",robotSn){}

void VelocityTrackingTest::singleStaticTestWithWait(uint16_t waitInterval){
    try{
        //Zero the FT Sensor before switching to NRT_CARTESIAN_MOTION_FORCE Mode (otherwise a fault occurs).
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        std::this_thread::sleep_for(std::chrono::seconds(2));

        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false}); //set all axes to be pure motion controlled

        // send a zero-velocity command to hold the current pose
        auto curr_pose = robot_->states().tcp_pose;
        const std::array<double,6> vel = {0,0,0,0,0,0};
        {
            std::lock_guard<std::mutex> lk(mtx_);
            lastTargetVelocity_ = vel;
        }
        robot_->SendCartesianMotionForce(curr_pose, {} , vel); //call with current position as target position and velocity as 0 to test for stability

        const auto start = std::chrono::steady_clock::now();
        // wait for the requested time (unless stop is requested)
        while (!stopRequested_ &&
               std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < waitInterval) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    catch(std::exception &e){
        std::cout<<"Error in SingleStaticTestWithWait : "<<e.what()<<std::endl;
        throw;
    }
}

void VelocityTrackingTest::loopStaticTest(uint16_t durationSeconds, uint16_t commandIntervalMs){
    try{
        const std::array<double, 6> zeroVelocity = {0, 0, 0, 0, 0, 0};
        {
            std::lock_guard<std::mutex> lk(mtx_);
            lastTargetVelocity_ = zeroVelocity;
        }

        //Zero the FT Sensor before switching to NRT_CARTESIAN_MOTION_FORCE Mode (otherwise a fault occurs).
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        std::this_thread::sleep_for(std::chrono::seconds(2));

        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false}); //set all axes to be pure motion controlled
        
        const auto testStartTime = std::chrono::steady_clock::now();
        auto nextTick = testStartTime;
        
        //run for the "durationSeconds" and call function every "commandIntervalMs"
        while (!stopRequested_ &&
               std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - testStartTime).count() < durationSeconds) {
            nextTick += std::chrono::milliseconds(commandIntervalMs);

            const auto targetPose = robot_->states().tcp_pose;
            robot_->SendCartesianMotionForce(targetPose, {}, zeroVelocity);

            std::this_thread::sleep_until(nextTick);
        }

    }
    catch(std::exception &e){
        std::cout<<"Error in SingleStaticTestWithWait : "<<e.what()<<std::endl;
        throw;
    }
}

void VelocityTrackingTest::periodicSineTest(double amp, double timePeriod, uint16_t numIterations)
{
    // Basic input validation before configuring the robot or logger
    if (timePeriod <= 0.0) {
        throw std::invalid_argument("timePeriod must be positive");
    }
    if (numIterations == 0) {
        return;
    }

    try {
        constexpr double kTwoPi = 6.283185307179586;
        const double omega = kTwoPi / timePeriod;

        //Zero the FT Sensor before switching to NRT_CARTESIAN_MOTION_FORCE Mode (otherwise a fault occurs).
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        std::this_thread::sleep_for(std::chrono::seconds(2));


        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});//set all axes to be pure motion controlled

        for (uint16_t iter = 0; iter < numIterations && !stopRequested_; ++iter) {
            const auto iterationStart = std::chrono::steady_clock::now();
            auto nextTick = iterationStart;

            while (!stopRequested_) {
                nextTick += std::chrono::milliseconds(1);
                const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - iterationStart).count();

                // X-axis sine velocity; other axes remain zero for this test
                std::array<double, 6> velocity = {
                    amp * std::sin(omega * t),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0};

                // Retrieve the latest pose to hold position while commanding velocity
                auto currentPose = robot_->states().tcp_pose;
                {
                    std::lock_guard<std::mutex> lk(mtx_);
                    lastTargetVelocity_ = velocity;
                }
                robot_->SendCartesianMotionForce(currentPose, {}, velocity);

                // Complete the iteration after a single period
                if (t >= timePeriod) {
                    break;
                }

                std::this_thread::sleep_until(nextTick);
            }
        }

    }
    catch (const std::exception& e) {
        std::cout << "Error in periodicSineTest : " << e.what() << std::endl;
        throw;
    }
}

void VelocityTrackingTest::performTest()
{
    #ifdef _WIN32
    struct TimerResolutionGuard
    {
        TimerResolutionGuard() { timeBeginPeriod(1); }
        ~TimerResolutionGuard() { timeEndPeriod(1); }
    } timerGuard;
    #endif
    
    try {
        // Helper to manage logging around each phase and stop early if requested
        auto runPhase = [&](const char* logName, auto&& phase) -> bool {
            if (stopRequested_) {
                return false;
            }
            
            startLogging(logName, 1);
            DataLogger::getInstance().setHeader("timestamp_ns,"
                "target_vx,target_vy,target_vz,target_wx,target_wy,target_wz,"
                "measured_vx,measured_vy,measured_vz,measured_wx,measured_wy,measured_wz,"
                "pose_x,pose_y,pose_z,pose_qw,pose_qx,pose_qy,pose_qz");
                try {
                    phase();
                }
                catch (...) {
                    stopLogging();
                    throw;
                }
                stopLogging();
                return !stopRequested_;
            };
            
            if (!runPhase("singleStaticTestWithWait.csv", [&]() { singleStaticTestWithWait(); })) {
                return;
            }
            
            if (!runPhase("loopStaticTest.csv", [&]() { loopStaticTest(); })) {
                return;
            }
            
            runPhase("periodicSineTest.csv", [&]() { periodicSineTest(0.1, 10.0, 2); });
        }
        catch (const std::exception& e) {
            std::cerr << "[VelocityTrackingTest] performTest error: " << e.what() << "\n";
        }
    }

void VelocityTrackingTest::log(std::ostream& out)
{
    std::array<double, 6> targetVel{};
    {
        std::lock_guard<std::mutex> lk(mtx_);
        targetVel = lastTargetVelocity_;
    }

    std::array<double, 6> measuredVel{};
    std::array<double, 7> tcpPose{};

    if (robot_) {
        try {
            auto states = robot_->states();
            measuredVel = states.tcp_vel;
            tcpPose = states.tcp_pose;
        }
        catch (const std::exception& e) {
            std::cerr << "[VelocityTrackingTest] Failed to read states in log: " << e.what() << "\n";
        }
    }

    out << flexiv::rdk::utility::Arr2Str(targetVel, 6, ",") << ","
        << flexiv::rdk::utility::Arr2Str(measuredVel, 6, ",") << ","
        << flexiv::rdk::utility::Arr2Str(tcpPose, 7, ",");
}
