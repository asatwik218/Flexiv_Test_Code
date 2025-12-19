#pragma once

#include "flexiv_tests/flexiv_robot_test.h"
#include <array>

class SimpleLoggingTest : public FlexivRobotTest {
public:
    SimpleLoggingTest(const std::string& robotSn);
    virtual ~SimpleLoggingTest() = default;

protected:
    void performTest() override;

private:
    // Test target poses
    std::array<double, 6> targetCartesianPose_mm_deg_;  // {x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg}
    std::array<double, 7> targetJointPose_deg_;         // {j1-j7 in degrees}
};
