#pragma once 

#include "core/ITest.h"
#include "core/data_logger.h"
#include <flexiv/rdk/robot.hpp>
#include <atomic>
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
    bool isFinished() const { return testFinished_; }
    void waitForCompletion();

protected:
    std::string testName_;
    std::string robotSn_;
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> testFinished_{true};
    std::unique_ptr<flexiv::rdk::Robot> robot_;
    std::thread thread_;
    std::mutex mtx_;
    DataLogger* logger_{nullptr};
    
    virtual void log(std::ostream& out);
    virtual void performTest() = 0;
    void startLogging(const std::string& filename, int intervalMs = 1);
    void stopLogging();

};
