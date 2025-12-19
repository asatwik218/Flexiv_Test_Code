#include "flexiv_tests/simple_logging_test.h"
#include <iostream>
#include <thread>
#include <chrono>

SimpleLoggingTest::SimpleLoggingTest(const std::string& robotSn)
    : FlexivRobotTest("SimpleLoggingTest", robotSn)
{
    // Define a target Cartesian pose in mm and degrees
    // Example: x=400mm, y=200mm, z=300mm, rx=180deg, ry=0deg, rz=90deg
    targetCartesianPose_mm_deg_ = {400.0, 200.0, 300.0, 180.0, 0.0, 90.0};

    // Define a target joint pose in degrees
    // Example: all joints at 0 degrees (home position)
    targetJointPose_deg_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void SimpleLoggingTest::performTest()
{
    std::cout << "[" << testName_ << "] Starting Simple Logging Test...\n";

    try
    {
        const std::array<double,6> poseA = {150,-685,40,0,180,0};
        const std::array<double,6> poseB = {-132, -685, 40, 0, 180, 0};

        // Test 2: Start logging with selected fields
        std::cout << "\n========== Test 2: Enum-Based Logging ==========\n";
        std::vector<LogField> fieldsToLog = {
            LogField::TCP_POSE,
            LogField::TCP_VEL,
            LogField::JOINT_POS,
            LogField::EXT_WRENCH_TCP
        };

        std::string logFile = "simple_logging_test.csv";
        startLogging(logFile, fieldsToLog, 10);  // Log every 10ms
        std::cout << "[" << testName_ << "] Logging started: " << logFile << "\n";
        std::cout << "[" << testName_ << "] Logging fields: TCP_POSE, TCP_VEL, JOINT_POS, EXT_WRENCH_TCP\n";

        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Zeroed FT sensor.\n";
        


        // Test 3: Move to a test position and check if reached
        std::cout << "\n========== Test 3: Cartesian Pose Checking ==========\n";
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false , false , false , false , false , false});
        robot_->SetForceControlFrame(flexiv::rdk::CoordType::TCP);

        robot_->SendCartesianMotionForce(convertPose_mmDeg_to_mQuat(poseA),{},{},0.05);

        while(!isCartesianPoseReached(poseA, {true , true , true , false , false , false}))
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        robot_->SendCartesianMotionForce(convertPose_mmDeg_to_mQuat(poseB),{},{},0.05);

        while(!isCartesianPoseReached(poseB, {true , true , true , false , false , false}))
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        stopLogging();

    }
    catch (const std::exception& e)
    {
        std::cerr << "[" << testName_ << "] Error: " << e.what() << '\n';
    }
}
