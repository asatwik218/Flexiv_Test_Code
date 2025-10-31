#pragma once 

#include "core/ITest.h"
#include "core/data_logger.h"
#include <flexiv/rdk/robot.hpp>
#include <mutex>
#include <thread>

class FlexivRobotTest : public ITest{

public:
    FlexivRobotTest(std::string testName, const std::string& robotSn);
    virtual ~FlexivRobotTest();

    bool initialise() override;
    TestResult runTest() override;
    void stop() override;
    void cleanup() override;

protected:
    std::string testName_;
    std::string robotSn_;
    std::atomic<bool> stopRequested_{false};
    std::unique_ptr<flexiv::rdk::Robot> robot_;
    std::thread thread_;
    std::mutex mtx_;
    DataLogger* logger_;
    
    virtual void log(std::ostream& out);
    virtual void performTest() = 0;
    void startLogging(const std::string& filename, int intervalMs = 1);
    void stopLogging();

};