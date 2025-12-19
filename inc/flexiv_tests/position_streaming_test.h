#pragma once

#include "flexiv_tests/flexiv_robot_test.h"
#include <array>

class PositionStreamingTest : public FlexivRobotTest
{
public:
    explicit PositionStreamingTest(const std::string& robotSn,
                                   const std::array<double, 6>& startingPose = {0.0, -685.0, 200.0, 0.0, 180.0, 0.0});
    ~PositionStreamingTest() override = default;

protected:
    void performTest() override;

private:
    // Motion limit constants (set to very high values to avoid built-in limits)
    static constexpr double MAX_LINEAR_VEL = 1.0;      // m/s
    static constexpr double MAX_ANGULAR_VEL = 10.0;     // rad/s
    static constexpr double MAX_LINEAR_ACC = 100.0;     // m/s²
    static constexpr double MAX_ANGULAR_ACC = 100.0;    // rad/s²

    // Starting pose for all tests (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
    std::array<double, 6> startingPose_mm_deg_;

    // Helper function to move to starting pose before each test
    void moveToStartingPose();

    // Test methods
    void constantVelocityTest(double velocity, uint16_t durationSeconds, uint16_t streamIntervalMs = 20,bool useCurrentPose=false );

    void varyingVelocityTest(double minVelocity, double maxVelocity, uint16_t durationSeconds, uint16_t streamIntervalMs = 20,bool useCurrentPose=false);

   void periodicCurveTest(double xAmplitude,double zAmplitude,double frequency,uint16_t numCycles,uint16_t streamIntervalMs);

   void stepPositionTest(double positionA_offset_m, double positionB_offset_m, double moveVelocity, uint16_t holdDuration_s, uint16_t numCycles, uint16_t streamIntervalMs = 20, bool useCurrentPose=false);

   void stepVelocityTest(double lowVelocity, double highVelocity, double velocityStepSize, uint16_t durationPerStep, uint16_t streamIntervalMs = 20, bool useCurrentPose=false);
};
