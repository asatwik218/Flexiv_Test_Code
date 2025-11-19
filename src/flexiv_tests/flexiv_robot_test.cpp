#include "flexiv_tests/flexiv_robot_test.h"

#include <flexiv/rdk/utility.hpp>
#include <filesystem>
#include <iostream>
#include <thread>

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

void FlexivRobotTest::log(std::ostream& out){

	flexiv::rdk::RobotStates states = robot_->states();

	out
	<< flexiv::rdk::utility::Arr2Str(states.ext_wrench_in_tcp, 3, ",") << ","
	<< flexiv::rdk::utility::Arr2Str(states.ext_wrench_in_world , 3, ",") << ","
	<< flexiv::rdk::utility::Arr2Str(states.tcp_pose , 5 , ",") << ","
	<< flexiv::rdk::utility::Arr2Str(states.tcp_vel , 5, ",") << ","
	<< flexiv::rdk::utility::Arr2Str(states.ft_sensor_raw,3, ",")
	;
}

void FlexivRobotTest::startLogging(const std::string& filename, int intervalMs)
{
    std::lock_guard<std::mutex> lk(mtx_);

    if (!logger_) {
        logger_ = &DataLogger::getInstance();
        logger_->setCallback([this](std::ostream& out) { this->log(out); });
    }

    // Build logs/<TestName>/<filename> relative to current working dir and ensure it exists
    std::filesystem::path logDir = std::filesystem::current_path() / "logs" / testName_;
    std::error_code ec;
    std::filesystem::create_directories(logDir, ec);
    if (ec) {
        throw std::runtime_error("Failed to create log directory: " + ec.message());
    }
    std::filesystem::path fullPath = logDir / filename;

    logger_->configure(fullPath.string(), intervalMs);
    logger_->start();

    std::cout << "[" << testName_ << "] Logging started: " << fullPath.string() << "\n";
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
