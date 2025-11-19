#include "flexiv_tests/velocity_tracking_test.h"

#include <flexiv/rdk/utility.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

#ifdef _WIN32
#include <Windows.h>
// RAII helper to raise Windows timer resolution for the duration of the test loop.
struct TimerResolutionGuard
{
    TimerResolutionGuard() { timeBeginPeriod(1); }
    ~TimerResolutionGuard() { timeEndPeriod(1); }
};
#endif

namespace
{
constexpr double kTwoPi = 6.283185307179586;

struct HoldOnceStep final : public VelocityTrackingTest::MotionStep
{
    bool update(double /*t*/, const flexiv::rdk::RobotStates& states, VelocityTrackingTest::Command& out) override
    {
        out.pose = states.tcp_pose;
        out.velocity.fill(0.0);
        out.wrench.fill(0.0);
        out.valid = true;
        return false; // single shot
    }
};

struct HoldLoopStep final : public VelocityTrackingTest::MotionStep
{
    explicit HoldLoopStep(double durationSec) : durationSec_(durationSec) {}

    bool update(double t, const flexiv::rdk::RobotStates& states, VelocityTrackingTest::Command& out) override
    {
        out.pose = states.tcp_pose;
        out.velocity.fill(0.0);
        out.wrench.fill(0.0);
        out.valid = true;
        return t < durationSec_;
    }

    double durationSec_;
};

struct ConstantVelocityStep final : public VelocityTrackingTest::MotionStep
{
    ConstantVelocityStep(std::array<double, 6> vel, double durationSec) : vel_(vel), durationSec_(durationSec) {}

    bool update(double t, const flexiv::rdk::RobotStates& states, VelocityTrackingTest::Command& out) override
    {
        out.pose = states.tcp_pose;
        out.velocity = vel_;
        out.wrench.fill(0.0);
        out.valid = true;
        return t < durationSec_;
    }

    std::array<double, 6> vel_;
    double durationSec_;
};

struct SineVelocityStep final : public VelocityTrackingTest::MotionStep
{
    SineVelocityStep(std::array<double, 6> amp, double freqHz, double durationSec)
        : amp_(amp), omega_(kTwoPi * freqHz), durationSec_(durationSec) {}

    bool update(double t, const flexiv::rdk::RobotStates& states, VelocityTrackingTest::Command& out) override
    {
        out.pose = states.tcp_pose;
        for (size_t i = 0; i < 6; ++i) {
            out.velocity[i] = amp_[i] * std::sin(omega_ * t);
        }
        out.wrench.fill(0.0);
        out.valid = true;
        return t < durationSec_;
    }

    std::array<double, 6> amp_;
    double omega_;
    double durationSec_;
};

} // namespace

// ---- Factory helpers ----
std::unique_ptr<VelocityTrackingTest::MotionStep> VelocityTrackingTest::makeHoldOnce()
{
    return std::make_unique<HoldOnceStep>();
}

std::unique_ptr<VelocityTrackingTest::MotionStep> VelocityTrackingTest::makeHoldLoop(double durationSec)
{
    return std::make_unique<HoldLoopStep>(durationSec);
}

std::unique_ptr<VelocityTrackingTest::MotionStep> VelocityTrackingTest::makeConstantVelocity(std::array<double, 6> vel, double durationSec)
{
    return std::make_unique<ConstantVelocityStep>(vel, durationSec);
}

std::unique_ptr<VelocityTrackingTest::MotionStep> VelocityTrackingTest::makeSineVelocity(std::array<double, 6> amp, double freqHz, double durationSec)
{
    return std::make_unique<SineVelocityStep>(amp, freqHz, durationSec);
}

// ---- Scenario builders ----
VelocityTrackingTest::Scenario VelocityTrackingTest::MakeStaticOnce()
{
    Scenario s;
    s.name = "static_once";
    s.steps.emplace_back(makeHoldOnce());
    return s;
}

VelocityTrackingTest::Scenario VelocityTrackingTest::MakeStaticLoop(double durationSec)
{
    Scenario s;
    s.name = "static_loop";
    s.steps.emplace_back(makeHoldLoop(durationSec));
    return s;
}

VelocityTrackingTest::Scenario VelocityTrackingTest::MakeConstantSweep(double vel_mps, double durationSec)
{
    Scenario s;
    s.name = "constant_sweep";
    s.steps.emplace_back(makeConstantVelocity({0.0, 0.0, vel_mps, 0.0, 0.0, 0.0}, durationSec));
    s.steps.emplace_back(makeConstantVelocity({0.0, 0.0, -vel_mps, 0.0, 0.0, 0.0}, durationSec));
    s.steps.emplace_back(makeConstantVelocity({vel_mps, 0.0, 0.0, 0.0, 0.0, 0.0}, durationSec));
    s.steps.emplace_back(makeConstantVelocity({-vel_mps, 0.0, 0.0, 0.0, 0.0, 0.0}, durationSec));
    s.steps.emplace_back(makeHoldOnce()); // settle
    return s;
}

VelocityTrackingTest::Scenario VelocityTrackingTest::MakeSineWaveZ(double amp_mps, double freqHz, double durationSec, double settleSec)
{
    Scenario s;
    s.name = "sine_wave_z";
    s.steps.emplace_back(makeSineVelocity({0.0, 0.0, amp_mps, 0.0, 0.0, 0.0}, freqHz, durationSec));
    if (settleSec > 0) {
        s.steps.emplace_back(makeHoldLoop(settleSec));
    }
    return s;
}

// ---- Constructors ----
VelocityTrackingTest::VelocityTrackingTest(const std::string& robotSn,
                                           std::array<double, 6> targetVelocity_mps,
                                           double durationSec,
                                           int logIntervalMs,
                                           std::string logFilename,
                                           bool zeroFtBeforeEachScenario)
    : FlexivRobotTest("VelocityTrackingTest", robotSn),
      scenarios_({Scenario{
          "constant_velocity",
          zeroFtBeforeEachScenario,
          {makeConstantVelocity(targetVelocity_mps, durationSec), makeHoldOnce()} // end with zero command
      }}),
      logIntervalMs_(logIntervalMs),
      logFilename_(std::move(logFilename))
{
}

VelocityTrackingTest::VelocityTrackingTest(const std::string& robotSn,
                                           std::vector<Scenario> scenarios,
                                           int logIntervalMs,
                                           std::string logFilename)
    : FlexivRobotTest("VelocityTrackingTest", robotSn),
      scenarios_(std::move(scenarios)),
      logIntervalMs_(logIntervalMs),
      logFilename_(std::move(logFilename))
{
}

VelocityTrackingTest::VelocityTrackingTest(const std::string& robotSn,
                                           Scenario scenario,
                                           int logIntervalMs,
                                           std::string logFilename)
    : FlexivRobotTest("VelocityTrackingTest", robotSn),
      scenarios_(std::vector<Scenario>{std::move(scenario)}),
      logIntervalMs_(logIntervalMs),
      logFilename_(std::move(logFilename))
{
}

// ---- Core loop ----
void VelocityTrackingTest::performTest()
{
#ifdef _WIN32
    TimerResolutionGuard timerGuard;
#endif

    if (scenarios_.empty()) {
        std::cerr << "[" << testName_ << "] No scenarios provided.\n";
        return;
    }

    if (!robot_) {
        std::cerr << "[" << testName_ << "] Robot handle is null; initialise() may have failed.\n";
        return;
    }

    std::cout << "[" << testName_ << "] Running " << scenarios_.size()
              << " scenario(s). Log file: " << logFilename_ << "\n";

    auto safeStopLogging = [this]() {
        try {
            stopLogging();
        }
        catch (const std::exception& e) {
            std::cerr << "[" << testName_ << "] Error stopping logger: " << e.what() << "\n";
        }
    };

    try {
        try {
            if (logger_) {
                logger_->setHeader("timestamp_ns,"
                                   "target_vx,target_vy,target_vz,target_wx,target_wy,target_wz,"
                                   "measured_vx,measured_vy,measured_vz,measured_wx,measured_wy,measured_wz,"
                                   "pose_x,pose_y,pose_z,pose_qw,pose_qx,pose_qy,pose_qz,"
                                   "scenario_idx,step_idx");
            }
            startLogging(logFilename_, logIntervalMs_);
        }
        catch (const std::exception& e) {
            std::cerr << "[" << testName_ << "] Failed to start logging: " << e.what() << "\n";
            stopRequested_ = true;
            return;
        }

        // Zero FT once up front for consistent baseline
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        std::this_thread::sleep_for(std::chrono::seconds(2));

        robot_->SetVelocityScale(100);
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis({false, false, false, false, false, false});

        for (size_t scenarioIdx = 0; scenarioIdx < scenarios_.size() && !stopRequested_; ++scenarioIdx) {
            const auto& scenario = scenarios_[scenarioIdx];
            currentScenarioIdx_.store(scenarioIdx);

            if (scenario.zeroFtBefore && scenarioIdx > 0) {
                robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
                robot_->ExecutePrimitive("ZeroFTSensor", {});
                std::this_thread::sleep_for(std::chrono::seconds(2));
                robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
                robot_->SetForceControlAxis({false, false, false, false, false, false});
            }

            std::cout << "[" << testName_ << "] Scenario " << (scenarioIdx + 1) << "/"
                      << scenarios_.size() << " : " << scenario.name << "\n";

            for (size_t stepIdx = 0; stepIdx < scenario.steps.size() && !stopRequested_; ++stepIdx) {
                auto& step = scenario.steps[stepIdx];
                currentStepIdx_.store(stepIdx);
                stepStartTime_ = std::chrono::steady_clock::now();

                const auto stepName = scenario.name + "_step_" + std::to_string(stepIdx);
                std::cout << "[" << testName_ << "]   Step " << (stepIdx + 1) << "/"
                          << scenario.steps.size() << " (" << stepName << ")\n";

                auto nextTick = std::chrono::steady_clock::now();
                while (!stopRequested_) {
                    const auto now = std::chrono::steady_clock::now();
                    const double t = std::chrono::duration<double>(now - stepStartTime_).count();
                    nextTick += std::chrono::milliseconds(1); // ~1 kHz

                    flexiv::rdk::RobotStates states;
                    try {
                        states = robot_->states();
                    }
                    catch (const std::exception& e) {
                        std::cerr << "[" << testName_ << "] Failed to read states: " << e.what() << "\n";
                        stopRequested_ = true;
                        break;
                    }

                    Command cmd;
                    const bool keepGoing = step->update(t, states, cmd);

                    if (cmd.valid) {
                        try {
                            auto pose = cmd.pose;
                            auto wrench = cmd.wrench;
                            auto vel = cmd.velocity;

                            {
                                std::lock_guard<std::mutex> lk(mtx_);
                                lastCommand_ = cmd;
                            }

                            robot_->SendCartesianMotionForce(pose, wrench, vel);
                        }
                        catch (const std::exception& e) {
                            std::cerr << "[" << testName_ << "] Command error: " << e.what() << "\n";
                            stopRequested_ = true;
                            break;
                        }
                    }

                    if (!keepGoing) {
                        break;
                    }

                    std::this_thread::sleep_until(nextTick);
                }
            }
        }

        safeStopLogging();

        if (stopRequested_) {
            std::cout << "[" << testName_ << "] Stopped before completing all scenarios.\n";
        }
        else {
            std::cout << "[" << testName_ << "] All scenarios completed.\n";
        }
    }
    catch (const std::exception& e) {
        safeStopLogging();
        stopRequested_ = true;
        std::cerr << "[" << testName_ << "] Error: " << e.what() << "\n";
    }
}

void VelocityTrackingTest::log(std::ostream& out)
{
    std::array<double, 6> targetVel{};
    std::array<double, 6> tcpVel{};
    std::array<double, 7> tcpPose{};
    size_t scenarioIdx = currentScenarioIdx_.load();
    size_t stepIdx = currentStepIdx_.load();

    {
        std::lock_guard<std::mutex> lk(mtx_);
        targetVel = lastCommand_.velocity;
    }

    if (robot_) {
        try {
            auto states = robot_->states();
            tcpVel = states.tcp_vel;
            tcpPose = states.tcp_pose;
        }
        catch (const std::exception& e) {
            std::cerr << "[" << testName_ << "] Failed to read states during log: " << e.what() << "\n";
        }
    }

    out << flexiv::rdk::utility::Arr2Str(targetVel, 6, ",")
        << flexiv::rdk::utility::Arr2Str(tcpVel, 6, ",")
        << flexiv::rdk::utility::Arr2Str(tcpPose, 7, ",")
        << scenarioIdx << "," << stepIdx;
}
