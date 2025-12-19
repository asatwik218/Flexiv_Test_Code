#include "flexiv_tests/flexiv_robot_test.h"

#include <flexiv/rdk/utility.hpp>
#include <filesystem>
#include <iostream>
#include <thread>
#include <cmath>
#include <sstream>

FlexivRobotTest::FlexivRobotTest(std::string testName, const std::string& robotSn):testName_(std::move(testName)), robotSn_(robotSn), stopRequested_(false)
{}

FlexivRobotTest::~FlexivRobotTest(){
    cleanup();
}

bool FlexivRobotTest::initialise(){

    try{
        robot_ = std::make_unique<flexiv::rdk::Robot>(robotSn_);
        
		// Clear fault on the connected robot if any
		while (robot_->fault()) {
			std::cout << "Fault occurred on the connected robot. Attempting to clear fault.\n";
			if (!robot_->ClearFault()) {
				std::cout << "[{}] Fault cannot be cleared.\n";
				return false;
			}
			std::cout<<"Fault on the connected robot is cleared.\n";
		}

		// Enable the robot. estop should be released
		robot_->Enable();
		std::cout << "Robot enabled.\n";

		// Wait for robot to become operational
		const int maxWait = 30; //max wait is 30 seconds;
		int waited = 0;
		while (!robot_->operational() && waited < maxWait) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
			waited++;
		}

		//even after 30 sec if the robot is not operational, throw an error.
		if (!robot_->operational()) {
			throw std::runtime_error("Robot did not become operational");
		}
		
		//logger setup;
		logger_ = &DataLogger::getInstance();
		logger_->setCallback([this](std::ostream& out){this->log(out);});
		
        return true;
    }
    catch(std::exception& e){
        std::cout<<"Error Occured while initialisation: "<<e.what()<<std::endl;
        return false;
    }
}

void FlexivRobotTest::cleanup()
{
    stop(); // ensures thread, robot, and logger are stopped safely

    std::lock_guard<std::mutex> lk(mtx_);

    if (robot_) {
        robot_.reset();
        std::cout << "[" << testName_ << "] Robot connection cleaned up.\n";
    }

    if (logger_) {
        logger_->stop();
        std::cout << "[" << testName_ << "] Logger cleaned up.\n";
    }
}

// New logging with field selection
void FlexivRobotTest::startLogging(const std::string& filename,
                                     const std::vector<LogField>& fields,
                                     int intervalMs)
{
    std::lock_guard<std::mutex> lk(mtx_);

    // Store selected fields
    logFields_ = fields;

    if (!logger_) {
        logger_ = &DataLogger::getInstance();
    }

    // Set callback to log selected fields
    logger_->setCallback([this](std::ostream& out) { this->logSelectedFields(out); });

    // Build logs/<TestName>/<filename> relative to current working dir and ensure it exists
    std::filesystem::path logDir = std::filesystem::current_path() / "logs" / testName_;
    std::error_code ec;
    std::filesystem::create_directories(logDir, ec);
    if (ec) {
        throw std::runtime_error("Failed to create log directory: " + ec.message());
    }
    std::filesystem::path fullPath = logDir / filename;

    // Generate header from selected fields
    std::stringstream headerStream;
    headerStream << "timestamp_ms";
    for (const auto& field : logFields_) {
        headerStream << ",";
        writeFieldHeaders(headerStream, field);
    }
    logger_->setHeader(headerStream.str());

    logger_->configure(fullPath.string(), intervalMs);
    logger_->start();

    std::cout << "[" << testName_ << "] Logging started: " << fullPath.string() << "\n";
    std::cout << "[" << testName_ << "] Logging " << logFields_.size() << " fields\n";
}

// Legacy logging interface (for backward compatibility)
void FlexivRobotTest::startLogging(const std::string& filename, int intervalMs)
{
    // Default to logging all measured values (no commanded values)
    std::vector<LogField> defaultFields = {
        LogField::EXT_WRENCH_TCP,
        LogField::EXT_WRENCH_WORLD,
        LogField::TCP_POSE,
        LogField::TCP_VEL,
        LogField::FT_SENSOR_RAW
    };

    startLogging(filename, defaultFields, intervalMs);
}

void FlexivRobotTest::stopLogging()
{
    DataLogger* loggerCopy = nullptr;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        loggerCopy = logger_;
    }

    if (loggerCopy) {
        loggerCopy->stop();
        std::cout << "[" << testName_ << "] Logging stopped.\n";
    }
    else {
        std::cout << "[" << testName_ << "] No active logger to stop.\n";
    }
}

TestResult FlexivRobotTest::runTest(){
	if(thread_.joinable()){
		std::cerr << "[ "<< testName_ << " ] Test already running.\n";
		return {false , "Test already running"};
	}

	stopRequested_ = false;
    testFinished_ = false;

	//launch test code
	thread_ = std::thread([this](){
		try{
			this->performTest();
		}
		catch(const std::exception& e){
			std::cerr<<"["<<testName_<<"] error : "<<e.what();
		}

		if(logger_) logger_->stop();
		std::cout << "[" << testName_ << "] Test thread finished.\n";
        testFinished_ = true;
	});

	return {true, "Test started asynchronously"};

}

void FlexivRobotTest::stop()
{
    {
        // Guard state so repeated stop() calls are safe and we don't block while joining
        std::lock_guard<std::mutex> lk(mtx_);

        if (stopRequested_) {
            std::cout << "[" << testName_ << "] Stop already requested.\n";
            return;
        }

        stopRequested_ = true;
    }

    std::cout << "[" << testName_ << "] Stop requested.\n";

    // Stop robot motion immediately
    if (robot_) {
        try {
            robot_->Stop();
            std::cout << "[" << testName_ << "] Robot stopped.\n";
        }
        catch (const std::exception& e) {
            std::cerr << "[" << testName_ << "] Error stopping robot: " << e.what() << "\n";
        }
    }

    // Stop logger (re-acquires mutex internally)
    stopLogging();

    if (thread_.joinable()) {
        thread_.join();
        std::cout << "[" << testName_ << "] Test thread joined.\n";
        testFinished_ = true;
    }
}

void FlexivRobotTest::waitForCompletion()
{
    if (thread_.joinable()) {
        thread_.join();
        testFinished_ = true;
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

double FlexivRobotTest::mmToM(double mm)
{
    return mm * 0.001;
}

double FlexivRobotTest::mToMM(double m)
{
    return m * 1000.0;
}

double FlexivRobotTest::degToRad(double deg)
{
    constexpr double kPi = 3.14159265358979323846;
    return (deg * (kPi / 180.0));
}

double FlexivRobotTest::radToDeg(double rad)
{
    constexpr double kPi = 3.14159265358979323846;
    return (rad / kPi * 180.0);
}

std::array<double, 4> FlexivRobotTest::eulerZYXToQuat(const std::array<double, 3>& euler_rad)
{
    // euler_rad = [roll (x), pitch (y), yaw (z)] in radians
    Eigen::AngleAxisd rollAngle(euler_rad[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(euler_rad[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(euler_rad[2], Eigen::Vector3d::UnitZ());

    // Compose rotations in ZYX order: R = yaw * pitch * roll
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    // Return as {w, x, y, z}
    return {q.w(), q.x(), q.y(), q.z()};
}

std::array<double, 3> FlexivRobotTest::quatToEulerZYX(const std::array<double, 4>& quat)
{
    return flexiv::rdk::utility::Quat2EulerZYX(quat);
}

std::array<double, 7> FlexivRobotTest::convertPose_mmDeg_to_mQuat(const std::array<double, 6>& pose_mm_deg)
{
    // Convert position from mm to meters
    double x_m = mmToM(pose_mm_deg[0]);
    double y_m = mmToM(pose_mm_deg[1]);
    double z_m = mmToM(pose_mm_deg[2]);

    // Convert Euler angles from degrees to radians, then to quaternion
    std::array<double, 3> euler_rad = {
        degToRad(pose_mm_deg[3]),
        degToRad(pose_mm_deg[4]),
        degToRad(pose_mm_deg[5])
    };

    std::array<double, 4> quat = eulerZYXToQuat(euler_rad);

    // Return as {x, y, z, qw, qx, qy, qz}
    return {x_m, y_m, z_m, quat[0], quat[1], quat[2], quat[3]};
}

std::array<double, 6> FlexivRobotTest::convertPose_mQuat_to_mmDeg(const std::array<double, 7>& pose_m_quat)
{
    // Convert position from meters to mm
    double x_mm = mToMM(pose_m_quat[0]);
    double y_mm = mToMM(pose_m_quat[1]);
    double z_mm = mToMM(pose_m_quat[2]);

    // Extract quaternion and convert to Euler angles in radians
    std::array<double, 4> quat = {
        pose_m_quat[3],  // qw
        pose_m_quat[4],  // qx
        pose_m_quat[5],  // qy
        pose_m_quat[6]   // qz
    };

    std::array<double, 3> euler_rad = quatToEulerZYX(quat);

    // Convert Euler angles from radians to degrees
    double rx_deg = radToDeg(euler_rad[0]);
    double ry_deg = radToDeg(euler_rad[1]);
    double rz_deg = radToDeg(euler_rad[2]);

    // Return as {x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg}
    return {x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg};
}

bool FlexivRobotTest::isCartesianPoseReached(const std::array<double, 6>& target_pose_mm_deg,
                                              const std::array<bool, 6>& axes_to_check,
                                              double threshold_m,
                                              double threshold_deg)
{
    // Convert target pose from mm/deg to m/quaternion
    std::array<double, 7> target_pose_m_quat = convertPose_mmDeg_to_mQuat(target_pose_mm_deg);

    // Get current TCP pose from robot
    auto current_pose = robot_->states().tcp_pose;

    // Debug: Print current and target positions
    auto current_pose_mm_deg = convertPose_mQuat_to_mmDeg(current_pose);

    // Check position (x, y, z)
    for (int i = 0; i < 3; ++i) {
        if (axes_to_check[i]) {
            double diff = std::abs(current_pose[i] - target_pose_m_quat[i]);
            if (diff > threshold_m) {
                return false;
            }
        }
    }

    // Check orientation (rx, ry, rz) - if any are enabled
    if (axes_to_check[3] || axes_to_check[4] || axes_to_check[5]) {
        // Convert both quaternions to Euler angles for comparison
        std::array<double, 4> current_quat = {
            current_pose[3],  // qw
            current_pose[4],  // qx
            current_pose[5],  // qy
            current_pose[6]   // qz
        };

        std::array<double, 4> target_quat = {
            target_pose_m_quat[3],  // qw
            target_pose_m_quat[4],  // qx
            target_pose_m_quat[5],  // qy
            target_pose_m_quat[6]   // qz
        };

        std::array<double, 3> current_euler_rad = quatToEulerZYX(current_quat);
        std::array<double, 3> target_euler_rad = quatToEulerZYX(target_quat);

        // Check each rotation axis (rx, ry, rz)
        for (int i = 0; i < 3; ++i) {
            if (axes_to_check[i + 3]) {  // axes_to_check[3,4,5] = rx,ry,rz
                double current_deg = radToDeg(current_euler_rad[i]);
                double target_deg = radToDeg(target_euler_rad[i]);

                // Handle angle wrapping (-180 to 180)
                double diff = std::abs(current_deg - target_deg);
                if (diff > 180.0) {
                    diff = 360.0 - diff;
                }

                const char* axis_names[] = {"rx", "ry", "rz"};

                if (diff > threshold_deg) {
                    return false;
                }
            }
        }
    }

    return true;
}

bool FlexivRobotTest::isJointPoseReached(const std::array<double, 7>& target_joint_pos_deg,
                                          const std::array<bool, 7>& joints_to_check,
                                          double threshold_deg)
{
    // Get current joint positions from robot (in radians)
    auto current_joint_pos_rad = robot_->states().q;

    // Check each joint
    for (int i = 0; i < 7; ++i) {
        if (joints_to_check[i]) {
            // Convert target from degrees to radians
            double target_rad = degToRad(target_joint_pos_deg[i]);

            // Convert current from radians to degrees for comparison
            double current_deg = radToDeg(current_joint_pos_rad[i]);
            double target_deg = target_joint_pos_deg[i];

            // Handle angle wrapping (-180 to 180)
            double diff = std::abs(current_deg - target_deg);
            if (diff > 180.0) {
                diff = 360.0 - diff;
            }

            if (diff > threshold_deg) {
                return false;
            }
        }
    }

    return true;
}

//robot related functions

void FlexivRobotTest::ZeroFTSensor(){
        try{
            const auto currMode = robot_->mode();
            robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
            robot_->ExecutePrimitive("ZeroFTSensor", {});
            std::this_thread::sleep_for(std::chrono::seconds(2));
            robot_->SwitchMode(currMode);
        }
        catch(const std::exception& e){
            std::cout<<"Exception"<<e.what()<<std::endl;
            throw;
        }
}


// ============================================================================
// Logging Helper Functions
// ============================================================================

void FlexivRobotTest::setCommandedTcpPose(const std::array<double, 7>& pose)
{
    std::lock_guard<std::mutex> lk(mtx_);
    cmdTcpPose_ = pose;
}

void FlexivRobotTest::setCommandedTcpVel(const std::array<double, 6>& vel)
{
    std::lock_guard<std::mutex> lk(mtx_);
    cmdTcpVel_ = vel;
}

void FlexivRobotTest::setCommandedJointPos(const std::vector<double>& pos)
{
    std::lock_guard<std::mutex> lk(mtx_);
    cmdJointPos_ = pos;
}

void FlexivRobotTest::logSelectedFields(std::ostream& out)
{
    for (size_t i = 0; i < logFields_.size(); ++i) {
        if (i > 0) out << ",";
        writeFieldValues(out, logFields_[i]);
    }
}

void FlexivRobotTest::writeFieldHeaders(std::ostream& out, LogField field)
{
    switch (field) {
        case LogField::TCP_POSE:
            out << "tcp_x,tcp_y,tcp_z,tcp_qw,tcp_qx,tcp_qy,tcp_qz";
            break;
        case LogField::TCP_VEL:
            out << "tcp_vel_x,tcp_vel_y,tcp_vel_z,tcp_vel_wx,tcp_vel_wy,tcp_vel_wz";
            break;
        case LogField::JOINT_POS:
            out << "joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6,joint_pos_7";
            break;
        case LogField::JOINT_VEL:
            out << "joint_vel_1,joint_vel_2,joint_vel_3,joint_vel_4,joint_vel_5,joint_vel_6,joint_vel_7";
            break;
        case LogField::EXT_WRENCH_TCP:
            out << "ext_wrench_tcp_fx,ext_wrench_tcp_fy,ext_wrench_tcp_fz,ext_wrench_tcp_mx,ext_wrench_tcp_my,ext_wrench_tcp_mz";
            break;
        case LogField::EXT_WRENCH_WORLD:
            out << "ext_wrench_world_fx,ext_wrench_world_fy,ext_wrench_world_fz,ext_wrench_world_mx,ext_wrench_world_my,ext_wrench_world_mz";
            break;
        case LogField::FT_SENSOR_RAW:
            out << "ft_raw_fx,ft_raw_fy,ft_raw_fz,ft_raw_mx,ft_raw_my,ft_raw_mz";
            break;
        case LogField::CMD_TCP_POSE:
            out << "cmd_tcp_x,cmd_tcp_y,cmd_tcp_z,cmd_tcp_qw,cmd_tcp_qx,cmd_tcp_qy,cmd_tcp_qz";
            break;
        case LogField::CMD_TCP_VEL:
            out << "cmd_tcp_vel_x,cmd_tcp_vel_y,cmd_tcp_vel_z,cmd_tcp_vel_wx,cmd_tcp_vel_wy,cmd_tcp_vel_wz";
            break;
        case LogField::CMD_JOINT_POS:
            out << "cmd_joint_pos_1,cmd_joint_pos_2,cmd_joint_pos_3,cmd_joint_pos_4,cmd_joint_pos_5,cmd_joint_pos_6,cmd_joint_pos_7";
            break;
    }
}

void FlexivRobotTest::writeFieldValues(std::ostream& out, LogField field) const
{
    auto states = robot_->states();

    switch (field) {
        case LogField::TCP_POSE:
            out << flexiv::rdk::utility::Arr2Str(states.tcp_pose, 5, ",");
            break;
        case LogField::TCP_VEL:
            out << flexiv::rdk::utility::Arr2Str(states.tcp_vel, 5, ",");
            break;
        case LogField::JOINT_POS:
            out << flexiv::rdk::utility::Vec2Str(states.q, 5, ",");
            break;
        case LogField::JOINT_VEL:
            out << flexiv::rdk::utility::Vec2Str(states.dq, 5, ",");
            break;
        case LogField::EXT_WRENCH_TCP:
            out << flexiv::rdk::utility::Arr2Str(states.ext_wrench_in_tcp, 3, ",");
            break;
        case LogField::EXT_WRENCH_WORLD:
            out << flexiv::rdk::utility::Arr2Str(states.ext_wrench_in_world, 3, ",");
            break;
        case LogField::FT_SENSOR_RAW:
            out << flexiv::rdk::utility::Arr2Str(states.ft_sensor_raw, 3, ",");
            break;
        case LogField::CMD_TCP_POSE:
            out << flexiv::rdk::utility::Arr2Str(cmdTcpPose_, 5, ",");
            break;
        case LogField::CMD_TCP_VEL:
            out << flexiv::rdk::utility::Arr2Str(cmdTcpVel_, 5, ",");
            break;
        case LogField::CMD_JOINT_POS:
            if (!cmdJointPos_.empty()) {
                out << flexiv::rdk::utility::Vec2Str(cmdJointPos_, 5, ",");
            } else {
                out << "0,0,0,0,0,0,0";  // Default if not set
            }
            break;
    }
}
