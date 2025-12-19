#include "flexiv_tests/step_sensing_test.h"

#include <iostream>
#include <flexiv/rdk/utility.hpp>

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

StepSensingTest::StepSensingTest(const std::string& robotSn,   
    std::array<double,6> startPos_mm_deg,
    std::array<double,6> endPos_mm_deg,
    std::vector<double> targetForces_N,
    std::vector<double> targetVelocities_mPsec
):FlexivRobotTest("StepSensingTest",robotSn){
      
    //store target parameters
    targetForces_ = targetForces_N;
    targetVelocities_ = targetVelocities_mPsec;

    // --- Convert start pose ---
    std::array<double,3> startTrans_m = {
        mmToM(startPos_mm_deg[0]),
        mmToM(startPos_mm_deg[1]),
        mmToM(startPos_mm_deg[2])
    };
    std::array<double,3> startEuler_rad = {
        degToRad(startPos_mm_deg[3]),
        degToRad(startPos_mm_deg[4]),
        degToRad(startPos_mm_deg[5])
    };
    std::array<double,4> startQuat = eulerZYXToQuat(startEuler_rad);

    // --- Convert end pose ---
    std::array<double,3> endTrans_m = {
        mmToM(endPos_mm_deg[0]),
        mmToM(endPos_mm_deg[1]),
        mmToM(endPos_mm_deg[2])
    };
    std::array<double,3> endEuler_rad = {
        degToRad(endPos_mm_deg[3]),
        degToRad(endPos_mm_deg[4]),
        degToRad(endPos_mm_deg[5])
    };
    std::array<double,4> endQuat = eulerZYXToQuat(endEuler_rad);

    // Combine into TCP pose arrays (x, y, z, qw, qx, qy, qz)
    startPose_ = { startTrans_m[0], startTrans_m[1], startTrans_m[2],startQuat[0], startQuat[1], startQuat[2], startQuat[3] };
    endPose_ = { endTrans_m[0], endTrans_m[1], endTrans_m[2],endQuat[0], endQuat[1], endQuat[2], endQuat[3] };
}


bool StepSensingTest::isMotionComplete(const std::array<double,7> targetPose_m_quat ){

    std::cout << "target: " << flexiv::rdk::utility::Arr2Str(targetPose_m_quat,5,",") <<std::endl;
    double threshold = 0.005;
    for(int i=0; i<3; i++){
        if( abs(targetPose_m_quat[i] - robot_->states().tcp_pose[i]) > threshold ) return false;
    }

    return true;

}

void StepSensingTest::performTest()
{
    std::cout << "[" << testName_ << "] Starting Step Sensing Test...\n";

    try
    {
        std::array<double,6> wrench = {0,0,0,0,0,0};
        
        //loop for all combination of velocities and forces.
        for(size_t i = 0 ; i<targetVelocities_.size(); i++)
        {

            if (stopRequested_) {
                std::cout << "[" << testName_ << "] Stop requested before iteration start. Exiting.\n";
                break;
            }

            for (size_t j = 0; j < targetForces_.size(); ++j)
            {

                if (stopRequested_) {
                    std::cout << "[" << testName_ << "] Stop requested before iteration start. Exiting.\n";
                    break;
                }

                std::cout << "\n========== Iteration " << (i * targetForces_.size() + j + 1) << " ==========\n";
                std::cout << "Target Force: " << targetForces_[j] << " N, Target Velocity: " << targetVelocities_[i] << " m/s\n";

                // Start logger for this iteration
                std::string logFile = std::to_string(i + 1) + "_" + std::to_string(targetVelocities_[i]) + "mPsec_" + std::to_string(targetForces_[j]) + "N.csv";
                startLogging(logFile, 1);
                std::cout << "[" << testName_ << "] Logger started: " << logFile << "\n";

                // STEP 1: Set up hybrid motion force control parameters
                std::cout << "[Step 1] Configuring hybrid motion-force mode...\n";
                robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
                robot_->SetForceControlFrame(flexiv::rdk::CoordType::WORLD);
                
                // STEP 2: Move to start pose
                std::cout << "[Step 2] Moving to start pose...\n";
                robot_->SetForceControlAxis({ false , false , false , false , false , false });
                robot_->SendCartesianMotionForce(startPose_, {}, {} , 0.031);
                while(!isMotionComplete(startPose_) || !robot_->stopped()){
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                std::cout << "[" << testName_ << "] Reached start pose.\n";

                // STEP 3: Zero Force-torque Sensor
                std::cout << "[Step 3] Zeroing Force-Torque Sensor...\n";
                robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
                robot_->ExecutePrimitive("ZeroFTSensor", {});
                std::this_thread::sleep_for(std::chrono::seconds(2));
                std::cout << "FT Sensor Zeroed\n";


                // // STEP 4: Contact the surface
                // std::cout << "[Step 4] Searching for surface contact...\n";
                robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
                auto contactPose = startPose_;
                contactPose[2] -= 0.5;
                robot_->SendCartesianMotionForce(contactPose, {}, {0,0,0,0,0,0}, 0.05);

                bool is_contacted = false;
                while (!is_contacted)
                {
                    if (stopRequested_) {
                        std::cout << "[" << testName_ << "] Stop requested during contact search. Exiting loop.\n";
                        break;
                    }

                    double val = robot_->states().ext_wrench_in_tcp[2];
                    if( val >7 || val < -7) {
                        is_contacted = true;
                        std::cout << "[" << testName_ << "] Contact detected. tcp_wrench: "
                                << val << " N\n";
                    }
                    std::cout << "[" << testName_ << "] tcp_wrench: " << val << " N\n";
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }


                // // STEP 5: Perform test motion
                // std::cout << "[Step 5] Performing test motion: applying "<< targetForces_[j] << " N @ " << targetVelocities_[i] << " m/s...\n";
                // robot_->SetForceControlFrame(flexiv::rdk::CoordType::TCP);
                // std::cout << flexiv::rdk::utility::Arr2Str(endPose_,3,",")<<"\n";
                // robot_->SetForceControlAxis({false , false , true , false , false , false});
                // robot_->SendCartesianMotionForce(endPose_, { 0,0,-targetForces_[j],0,0,0 }, { 0,0,0,0,0,0 }, targetVelocities_[i]);

                // bool isReached = false;
                // while (!isReached)
                // {
                //     if (stopRequested_) {
                //         std::cout << "[" << testName_ << "] Stop requested during motion. Exiting loop.\n";
                //         break;
                //     }

                //     auto tcp_pose = robot_->states().tcp_pose;
                //     if ( std::abs(tcp_pose[0] - endPose_[0]) < 0.005 && std::abs(tcp_pose[1] - endPose_[1]) < 0.005 ) {
                //         isReached = true;
                //         std::cout << "[" << testName_ << "] End pose reached.\n";
                //     }

                //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
                // }

                // Stop logger after iteration
                stopLogging();
                std::cout << "[" << testName_ << "] Logger stopped for iteration " << (i + 1) << "\n";
            }
        }
        std::cout << "[" << testName_ << "] Step Sensing Test completed successfully.\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "[" << testName_ << "] Error: " << e.what() << '\n';
    }
}