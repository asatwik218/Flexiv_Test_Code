#include "flexiv_tests/path_repeatability_test.h"
#include <flexiv/rdk/utility.hpp>
#include <flexiv/rdk/robot.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <chrono>
#include <cmath>

#define MOVEL 1
#define SENDCARTESIANMOTIONFORCE 0

PathRepeatabilityTest::PathRepeatabilityTest(const std::string& robotSn)
    : FlexivRobotTest("PathRepeatabilityTest", robotSn)
{}

inline double PathRepeatabilityTest::degToRad(double deg) const
{
    constexpr double kPi = 3.14159265358979323846;
    return (deg * (kPi / 180.0));
}

inline double PathRepeatabilityTest::radToDeg(double rad) const
{
    constexpr double kPi = 3.14159265358979323846;
    return (rad / kPi * 180.0);
}

std::array<double, 4> PathRepeatabilityTest::eulerZYXToQuat(const std::array<double, 3>& euler) const
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

std::array<double, 3> PathRepeatabilityTest::quatToEulerZYX(const std::array<double, 4>& quat) const
{
    return flexiv::rdk::utility::Quat2EulerZYX(quat);
}

int PathRepeatabilityTest::transformPoseFromRefToWorld(
    const std::array<double, 6>& poseInRefFrame_mm_deg,
    const std::array<double, 6>& refCoordPose_mm_deg,
    std::array<double, 6>& poseInWorld_mm_deg) const
{
    try
    {
        // --- 1. Build Isometry for world -> Reference frame (refCoordPose) ---
        Eigen::Isometry3d T_world_ref = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd rollB(degToRad(refCoordPose_mm_deg[3]), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchB(degToRad(refCoordPose_mm_deg[4]), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawB(degToRad(refCoordPose_mm_deg[5]), Eigen::Vector3d::UnitZ());
        T_world_ref.linear() = (yawB * pitchB * rollB).toRotationMatrix();
        T_world_ref.translation() = Eigen::Vector3d(
            mmToM(refCoordPose_mm_deg[0]),
            mmToM(refCoordPose_mm_deg[1]),
            mmToM(refCoordPose_mm_deg[2]));

        // --- 2. Build Isometry for referenceFrame -> TCP (poseInRefFrame) ---
        Eigen::Isometry3d T_ref_tcp = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd rollA(degToRad(poseInRefFrame_mm_deg[3]), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchA(degToRad(poseInRefFrame_mm_deg[4]), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawA(degToRad(poseInRefFrame_mm_deg[5]), Eigen::Vector3d::UnitZ());
        T_ref_tcp.linear() = (yawA * pitchA * rollA).toRotationMatrix();
        T_ref_tcp.translation() = Eigen::Vector3d(
            mmToM(poseInRefFrame_mm_deg[0]),
            mmToM(poseInRefFrame_mm_deg[1]),
            mmToM(poseInRefFrame_mm_deg[2]));

        // --- 3. Compute world -> TCP transform ---
        // T_world_tcp = T_world_ref * T_ref_tcp
        Eigen::Isometry3d T_world_tcp = T_world_ref * T_ref_tcp;

        // --- 4. Extract translation in mm ---
        Eigen::Vector3d p_m = T_world_tcp.translation();
        poseInWorld_mm_deg[0] = mToMM(p_m.x());
        poseInWorld_mm_deg[1] = mToMM(p_m.y());
        poseInWorld_mm_deg[2] = mToMM(p_m.z());

        // --- 5. Extract proper ZYX (yaw-pitch-roll) Euler angles ---
        const Eigen::Matrix3d& R = T_world_tcp.linear();
        double roll, pitch, yaw;

        double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
        bool singular = sy < 1e-6;

        if (!singular)
        {
            pitch = std::atan2(-R(2, 0), sy);
            roll = std::atan2(R(2, 1), R(2, 2));
            yaw = std::atan2(R(1, 0), R(0, 0));
        }
        else
        {
            // Gimbal lock: cos(pitch) nearly 0
            pitch = std::atan2(-R(2, 0), sy);
            roll = 0.0;
            yaw = std::atan2(-R(0, 1), R(1, 1));
        }

        poseInWorld_mm_deg[3] = radToDeg(roll);
        poseInWorld_mm_deg[4] = radToDeg(pitch);
        poseInWorld_mm_deg[5] = radToDeg(yaw);

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[PathRepeatabilityTest] Exception in transformPoseFromRefToWorld: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "[PathRepeatabilityTest] Unknown exception in transformPoseFromRefToWorld" << std::endl;
        return 1;
    }
}

std::array<double, 7> PathRepeatabilityTest::convertWaypoint(const std::array<double, 6>& waypoint_mm_deg) const
{
    std::array<double, 7> pose = {};

    // Convert position from mm to m
    pose[0] = mmToM(waypoint_mm_deg[0]);
    pose[1] = mmToM(waypoint_mm_deg[1]);
    pose[2] = mmToM(waypoint_mm_deg[2]);

    // Convert orientation from deg to rad, then to quaternion
    std::array<double, 4> orientationQuat = eulerZYXToQuat({
        degToRad(waypoint_mm_deg[3]),  // roll
        degToRad(waypoint_mm_deg[4]),  // pitch
        degToRad(waypoint_mm_deg[5])   // yaw
    });

    pose[3] = orientationQuat[0];  // qw
    pose[4] = orientationQuat[1];  // qx
    pose[5] = orientationQuat[2];  // qy
    pose[6] = orientationQuat[3];  // qz

    return pose;
}


void PathRepeatabilityTest::performTest()
{
    try {
        // Robot initialization
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        std::this_thread::sleep_for(std::chrono::seconds(2));
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});

        // Reference frame pose (camera_robot_base in world frame) - TODO: Update these values
        std::array<double, 6> refCoordPose_mm_deg = {-270, 50, 225, 0, 0, 180};  // [x, y, z, roll, pitch, yaw]

        // Waypoints in camera_robot_base frame (mm and degrees)
        std::vector<std::array<double, 6>> waypointsInRefFrame = {
            {-1099, 700, -199, -179, 0, -117},
            {-449, 748, -193, -179, 0, -16},
            {-1099, 700, -199, -179, 0, -117},
            {-392, 731, -148, -179, 0, -119},
            {-1099, 700, -199, -179, 0, -117},
            {-422, 779, -132, -179, 0, 54}
        };

        // Test parameters
        uint16_t numRepetitions = 10;
        double maxLinearVel_mmPsec = 100.0;
        double maxAngularVel_degPsec = 30.0;
        double maxLinearAcc_mmPsec2 = 1000.0;
        double maxAngularAcc_degPsec2 = 300.0;
        double positionTolerance_mm = 0.5;
        uint16_t timeoutSeconds = 100;

        // Convert units
        double maxLinearVel_m = mmToM(maxLinearVel_mmPsec);
        double maxAngularVel_rad = degToRad(maxAngularVel_degPsec);
        double maxLinearAcc_m = mmToM(maxLinearAcc_mmPsec2);
        double maxAngularAcc_rad = degToRad(maxAngularAcc_degPsec2);
        double positionTolerance_m = mmToM(positionTolerance_mm);

        std::cout << "[PathRepeatabilityTest] Starting test with "<< waypointsInRefFrame.size() << " waypoints, "<< numRepetitions << " repetitions" << std::endl;

        std::array<double, 6> targetWrench = {0, 0, 0, 0, 0, 0};
        std::array<double, 6> targetVelocity = {0, 0, 0, 0, 0, 0};
        constexpr uint16_t controlLoopMs = 10;
        
        // Change Mode
        #if SENDCARTESIANMOTIONFORCE == 1 
            robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        #elif MOVEL == 1
            robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        #else 
            robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        #endif

        // Execute repetitions
        for (uint16_t rep = 0; rep < numRepetitions && !stopRequested_; ++rep) {
            std::cout << "[PathRepeatabilityTest] Repetition " << (rep + 1)<< " / " << numRepetitions << std::endl;
            
            // Execute each waypoint
            for (size_t i = 0; i < waypointsInRefFrame.size() && !stopRequested_; ++i) {

                #if SENDCARTESIANMOTIONFORCE == 1

                    // Transform waypoint from reference frame to world frame
                    std::array<double, 6> waypointInWorld_mm_deg;
                    int result = transformPoseFromRefToWorld(
                        waypointsInRefFrame[i],
                        refCoordPose_mm_deg,
                        waypointInWorld_mm_deg
                    );

                    if (result != 0) {
                        std::cerr << "[PathRepeatabilityTest] Failed to transform waypoint "<< (i + 1) << std::endl;
                        return;
                    }

                    // Print pose in both frames
                    std::cout << "[PathRepeatabilityTest] Waypoint " << (i + 1) << " / " << waypointsInRefFrame.size() << std::endl;
                    std::cout << "  Ref Frame  : ["
                            << waypointsInRefFrame[i][0] << ", "
                            << waypointsInRefFrame[i][1] << ", "
                            << waypointsInRefFrame[i][2] << ", "
                            << waypointsInRefFrame[i][3] << ", "
                            << waypointsInRefFrame[i][4] << ", "
                            << waypointsInRefFrame[i][5] << "] mm/deg" << std::endl;
                    std::cout << "  World Frame: ["
                            << waypointInWorld_mm_deg[0] << ", "
                            << waypointInWorld_mm_deg[1] << ", "
                            << waypointInWorld_mm_deg[2] << ", "
                            << waypointInWorld_mm_deg[3] << ", "
                            << waypointInWorld_mm_deg[4] << ", "
                            << waypointInWorld_mm_deg[5] << "] mm/deg" << std::endl;

                    // Convert to m/quat format
                    auto targetPose = convertWaypoint(waypointInWorld_mm_deg);
                    std::cout << "  Robot Format: ["
                            << targetPose[0] << ", "
                            << targetPose[1] << ", "
                            << targetPose[2] << ", "
                            << targetPose[3] << ", "
                            << targetPose[4] << ", "
                            << targetPose[5] << ", "
                            << targetPose[6] << "] m/quat" << std::endl;
                    
                    //start motion
                    robot_->SendCartesianMotionForce(
                        targetPose,
                        targetWrench,
                        targetVelocity,
                        maxLinearVel_m,
                        maxAngularVel_rad,
                        maxLinearAcc_m,
                        maxAngularAcc_rad
                    );

                    while(true){
                        // Check if reached target (position only)
                        auto currentPose = robot_->states().tcp_pose;
                        double posError = std::sqrt(
                            std::pow(targetPose[0] - currentPose[0], 2) +
                            std::pow(targetPose[1] - currentPose[1], 2) +
                            std::pow(targetPose[2] - currentPose[2], 2)
                        );

                        if(posError <= positionTolerance_m){
                            break;
                        }
                    }

                #elif MOVEL == 1 //MoveL
                   
                    flexiv::rdk::Coord targetPose(
                        { mmToM(waypointsInRefFrame[i][0]), mmToM(waypointsInRefFrame[i][1]), mmToM(waypointsInRefFrame[i][2]) }, //X,Y,Z in m
                        { waypointsInRefFrame[i][3], waypointsInRefFrame[i][4], waypointsInRefFrame[i][5] }, // Rx, Ry ,Rz in deg
                        { "WORK", "CAMERA_ROBOT_BASE" }
                    ); //Ref Frame
                    robot_->ExecutePrimitive("MoveL",
                    {
                        {"target", targetPose},
                        {"vel", 0.2},
                        {"enableSixAxisJntCtrl",0},
                        {"enableFixRefJntPos",1},
                        {"refJntPos", std::vector<double>{26,63,-118,+103,-9,+40,-148,0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}

                    });
            
                    // Wait for reached target
                    while (!std::get<int>(robot_->primitive_states()["reachedTarget"])) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(controlLoopMs));
                    }
                    
                #else
                    flexiv::rdk::Coord targetPose(
                        { mmToM(waypointsInRefFrame[i][0]), mmToM(waypointsInRefFrame[i][1]), mmToM(waypointsInRefFrame[i][2]) }, //X,Y,Z in m
                        { waypointsInRefFrame[i][3], waypointsInRefFrame[i][4], waypointsInRefFrame[i][5] }, // Rx, Ry ,Rz in deg
                        { "WORK", "CAMERA_ROBOT_BASE" }
                    ); //Ref Frame
                    robot_->ExecutePrimitive("MovePTP",
                    {
                        {"target", targetPose},
                        {"jntVelScale", 25},
                    });
            
                    // Wait for reached target
                    while (!std::get<int>(robot_->primitive_states()["reachedTarget"])) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(controlLoopMs));
                        }
                #endif
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        std::cout << "[PathRepeatabilityTest] Test completed successfully" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[PathRepeatabilityTest] Error: " << e.what() << std::endl;
        throw;
    }
}
