#pragma once

#include "flexiv_tests/flexiv_robot_test.h"
#include <array>
#include <vector>

class ResponseTimeTest : public FlexivRobotTest
{
public:
    explicit ResponseTimeTest(const std::string& robotSn);
    ~ResponseTimeTest() override = default;

protected:
    void performTest() override;
    void log(std::ostream& out) override;

private:
    // Core test execution method
    void executeTest();

    // Thread-safe state for logging (protected by inherited mtx_)
    struct CommandState {
        std::array<double, 7> targetPose{};
        int commandIndex{0};
        int commandDelayMs{0};
        int64_t commandTimestamp_ns{0};
    };
    CommandState lastCommandState_;

    // Test configuration - USER WILL CUSTOMIZE THESE VALUES
    // Position array with delay for each position: {pose, delay_ms}
    // Format: {{x, y, z, qw, qx, qy, qz}, delay_ms}
    std::vector<std::pair<std::array<double, 7>, int>> testPositions_ = {
        {{-0.600,-0.595,0.128,0,0,-1,0}, 0},   
        {{0.680,-0.595,0.128,0,0,-1,0}, 20},   
        {{-0.600,-0.595,0.128,0,0,-1,0}, 15},   
        {{+0.667,-0.595,0.128,0,0,-1,0}, 5},  
    };

    // Motion parameters - user will customize
    double maxLinearVel_ = 1;        // m/s (high velocity)
    double maxAngularVel_ = 3.14;      // rad/s (high angular velocity)
    double maxLinearAcc_ = 20.0;       // m/s^2 (maximum acceleration)
    double maxAngularAcc_ = 31.4;      // rad/s^2 (maximum angular acceleration)
    uint16_t settlingTimeMs_ = 5000;   // milliseconds to wait after all commands
};
