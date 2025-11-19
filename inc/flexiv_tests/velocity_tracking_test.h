#pragma once

#include "flexiv_tests/flexiv_robot_test.h"

#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Exercise velocity tracking with pluggable motion steps (scenarios).
 *
 * The test acts as a scheduler: it runs a list of MotionStep instances that each
 * emit a pose/velocity command given current state and elapsed time. Adding new
 * tests (static hold once, static hold looped, sine wave, etc.) only requires
 * adding a new MotionStep implementation and wiring it into the scenario list.
 */
class VelocityTrackingTest : public FlexivRobotTest
{
public:
    struct Command
    {
        std::array<double, 7> pose{};
        std::array<double, 6> wrench{};
        std::array<double, 6> velocity{};
        bool valid{false};
    };

    struct MotionStep
    {
        virtual ~MotionStep() = default;
        /**
         * @param t            elapsed seconds in this step
         * @param states       latest robot states
         * @param outCommand   command to send this tick
         * @return true to continue this step; false when the step is finished
         */
        virtual bool update(double t, const flexiv::rdk::RobotStates& states, Command& outCommand) = 0;
    };

    struct Scenario
    {
        std::string name;
        bool zeroFtBefore{false};
        std::vector<std::unique_ptr<MotionStep>> steps;
    };

    // Predefined scenario builders (add more without touching the scheduler)
    static Scenario MakeStaticOnce();
    static Scenario MakeStaticLoop(double durationSec);
    static Scenario MakeConstantSweep(double vel_mps, double durationSec);
    static Scenario MakeSineWaveZ(double amp_mps, double freqHz, double durationSec, double settleSec = 1.0);

    // Convenience: single constant-velocity segment
    VelocityTrackingTest(const std::string& robotSn,
                         std::array<double, 6> targetVelocity_mps,
                         double durationSec,
                         int logIntervalMs = 1,
                         std::string logFilename = "velocity_tracking.csv",
                         bool zeroFtBeforeEachScenario = false);

    // General: provide scenarios directly
    VelocityTrackingTest(const std::string& robotSn,
                         std::vector<Scenario> scenarios,
                         int logIntervalMs = 1,
                         std::string logFilename = "velocity_tracking.csv");

    // Single scenario convenience
    VelocityTrackingTest(const std::string& robotSn,
                         Scenario scenario,
                         int logIntervalMs = 1,
                         std::string logFilename = "velocity_tracking.csv");

protected:
    void performTest() override;
    void log(std::ostream& out) override;

private:
    // Built-in step helpers
    static std::unique_ptr<MotionStep> makeHoldOnce();
    static std::unique_ptr<MotionStep> makeHoldLoop(double durationSec);
    static std::unique_ptr<MotionStep> makeConstantVelocity(std::array<double, 6> vel, double durationSec);
    static std::unique_ptr<MotionStep> makeSineVelocity(std::array<double, 6> amp, double freqHz, double durationSec);

    std::vector<Scenario> scenarios_;
    const int logIntervalMs_;
    const std::string logFilename_;

    std::atomic<size_t> currentScenarioIdx_{std::numeric_limits<size_t>::max()};
    std::atomic<size_t> currentStepIdx_{std::numeric_limits<size_t>::max()};
    std::chrono::steady_clock::time_point stepStartTime_;
    Command lastCommand_;
};
