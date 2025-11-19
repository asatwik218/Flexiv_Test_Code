#pragma once

#include "flexiv_tests/flexiv_robot_test.h"

#include <array>
#include <string>

/**
 * @brief Simple velocity tracking test with three fixed routines:
 *        static once, static loop, and periodic sine motion.
 */
class VelocityTrackingTest : public FlexivRobotTest
{
public:
    VelocityTrackingTest(const std::string& robotSn);

protected:
    void performTest() override;
    void log(std::ostream& out) override;

private:
    void singleStaticTestWithWait(uint16_t waitInterval = 30);
    void loopStaticTest(uint16_t durationSeconds = 30, uint16_t commandIntervalMs = 10);
    void periodicSineTest(double amp, double timePeriod , uint16_t numIterations=1);

    std::array<double, 6> lastTargetVelocity_{};
};
