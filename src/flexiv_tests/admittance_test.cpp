#include "flexiv_tests/admittance_test.h"
#ifdef _WIN32
  #include <windows.h>
  #include <mmsystem.h>
#endif
// Constructor
AdmittanceTest::AdmittanceTest(
    const std::string& robotSn,
    bool isAdmittance,
    uint16_t testDuration,
    const std::array<double, 6>& startingPose_mm_deg,
    const std::array<double, 6>& endPose_mm_deg,
    double pos_m,
    double pos_k,
    const std::array<double, 3>& high_end)
    : FlexivRobotTest("AdmittanceTest", robotSn),
      m_startingPose_mm_deg(startingPose_mm_deg),
      m_endPose_mm_deg(endPose_mm_deg),
      m_isAdmittance(isAdmittance),
      m_testDuration(testDuration),
      m_pos_m(pos_m),
      m_pos_k(pos_k)
{
    // Compute damping (critical damping)
    m_pos_d = 2.0 * std::sqrt(m_pos_m * m_pos_k);

    // Build diagonal mass/stiffness/damping matrices (direct initialization)
    m_K = Eigen::Vector3d(m_pos_k, m_pos_k, m_pos_k).asDiagonal();
    m_D = Eigen::Vector3d(m_pos_d, m_pos_d, m_pos_d).asDiagonal();
    m_InvM = Eigen::Vector3d(1.0/m_pos_m, 1.0/m_pos_m, 1.0/m_pos_m).asDiagonal();

    // pack force thresholds
    m_high_end = Eigen::Vector3d(high_end[0], high_end[1], high_end[2]);
    m_low_end = -m_high_end;

    m_desiredPosition = Eigen::Vector3d(mmToM(endPose_mm_deg[0]), mmToM(endPose_mm_deg[1]), mmToM(endPose_mm_deg[2]) );
}

// -------------------- Helper functions -------------------- //
inline Eigen::Vector3d AdmittanceTest::applySmoothDeadZone3(
    const Eigen::Vector3d& input_s,
    const Eigen::Vector3d& low_end,
    const Eigen::Vector3d& high_end,
    double sw_gn)
{
    Eigen::Vector3d output;
    for (int i = 0; i < 3; ++i)
    {
        const double active_pve = 0.5 * (std::tanh(sw_gn * ( input_s[i] - high_end[i])) + 1.0);
        const double active_nve = 0.5 * (std::tanh(sw_gn * (-input_s[i] +  low_end[i])) + 1.0);

        output[i] = active_nve * (input_s[i] - low_end[i])
                  + active_pve * (input_s[i] - high_end[i]);
    }
    return output;
}

inline Eigen::Vector3d AdmittanceTest::applySmoothSaturation(const Eigen::Vector3d& input, double max_limit)
{
    constexpr double sw_gn = 200.0;
    constexpr double delta = 0.001;

    const double input_norm = input.norm();
    if (input_norm <= std::numeric_limits<double>::epsilon() || max_limit <= 0.0) {
        return Eigen::Vector3d::Zero();
    }

    const double sat_activation = 0.5 * (std::tanh(sw_gn * (input_norm - max_limit - delta)) + 1.0);
    const Eigen::Vector3d direction = input / input_norm;

    const double saturated_norm = (1.0 - sat_activation) * input_norm + sat_activation * max_limit;
    return direction * saturated_norm;
}

Eigen::Vector3d AdmittanceTest::generateMotionDynamics(
    const Eigen::Vector3d& cur_position,
    const Eigen::Vector3d& cur_lin_vel_ee,
    const Eigen::Vector3d& adm_force_ee,
    double dt) const
{
    // Use member variables directly
    const Eigen::Vector3d pos_error = cur_position - m_desiredPosition;
    const double transmit_wrench = m_isAdmittance ? 1.0 : 0.0;

    // Second-order dynamics (mass-spring-damper) - direct matrix access, no map lookup
    const Eigen::Vector3d stiffness_force = m_K * pos_error;
    const Eigen::Vector3d equival_des_lin_vel = -m_D.inverse() * (stiffness_force - transmit_wrench * adm_force_ee);
    const Eigen::Vector3d sat_equival_des_lin_vel = applySmoothSaturation(equival_des_lin_vel, 0.1);
    const Eigen::Vector3d equival_damping_force = m_D * (cur_lin_vel_ee - sat_equival_des_lin_vel);
    const Eigen::Vector3d des_acceleration_lin = -m_InvM * equival_damping_force;
    const Eigen::Vector3d des_velocity_lin = cur_lin_vel_ee + des_acceleration_lin * dt;

    return applySmoothSaturation(des_velocity_lin, m_maxLinearVel);
}

// -------------------- moveToStartingPose -------------------- //
void AdmittanceTest::moveToStartingPose()
{
    std::cout << "[AdmittanceTest] Moving to starting pose: x=" << m_startingPose_mm_deg[0]
              << " mm, y=" << m_startingPose_mm_deg[1] << " mm, z=" << m_startingPose_mm_deg[2]
              << " mm, rx=" << m_startingPose_mm_deg[3] << " deg, ry=" << m_startingPose_mm_deg[4]
              << " deg, rz=" << m_startingPose_mm_deg[5] << " deg\n";

    // Set robot mode and disable force axes by default (you may alter axes if needed)
    robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
    robot_->SetForceControlAxis({false, false, false, false, false, false});

    // convertPose_mmDeg_to_mQuat is assumed provided by your base/helpers
    auto target_pose_m_quat = convertPose_mmDeg_to_mQuat(m_startingPose_mm_deg);
    robot_->SendCartesianMotionForce(target_pose_m_quat, {}, {}, 0.08);

    // Wait until starting pose is reached
    while (!isCartesianPoseReached(m_startingPose_mm_deg, {true, true, true, false, false, false})) {
        if (stopRequested_) {
            std::cout << "[AdmittanceTest] Stop requested during move to starting pose\n";
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot_->Stop();
    std::cout << "[AdmittanceTest] Starting pose reached\n";

}

// -------------------- admittanceControlLoop -------------------- //
void AdmittanceTest::admittanceControlLoop(uint16_t streamIntervalMs)
{
    const double controlTimestep = static_cast<double>(streamIntervalMs) / 1000.0;

    std::cout << "[AdmittanceTest] Starting admittance control loop ("
              << (1000.0 / streamIntervalMs) << " Hz, " << streamIntervalMs << " ms interval)\n";

#ifdef _WIN32
    timeBeginPeriod(1);
#endif

    try {
        auto loop_start = std::chrono::steady_clock::now();
        auto next_tick = loop_start;
        const auto dt_chrono = std::chrono::milliseconds(streamIntervalMs);

        // Get initial robot state
        auto states = robot_->states();

        // Initialize pose and velocity
        std::array<double, 7> target_pose = states.tcp_pose;
        std::array<double,7> initial_pose = target_pose;

        // Timing statistics
        double max_loop_time = 0.0;
        double sum_loop_time = 0.0;
        uint64_t loop_count = 0;

        std::cout << "[AdmittanceTest] Control loop active.\n";

        while (!stopRequested_) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - loop_start).count();

            if (elapsed >= static_cast<double>(m_testDuration)) {
                std::cout << "[AdmittanceTest] Test duration elapsed (" << m_testDuration << "s)\n";
                break;
            }

            next_tick += dt_chrono;

            // Read current robot state
            states = robot_->states();

            Eigen::Vector3d currentPosition(states.tcp_pose[0], states.tcp_pose[1], states.tcp_pose[2]);

            Eigen::Vector3d m_currentVelocity(states.tcp_vel[0], states.tcp_vel[1], states.tcp_vel[2]);

            Eigen::Vector3d measuredForce(-states.ext_wrench_in_tcp[0],states.ext_wrench_in_tcp[1], -states.ext_wrench_in_tcp[2]);

            Eigen::Vector3d admittanceForce = applySmoothDeadZone3(measuredForce, m_low_end, m_high_end, m_switchingGain);

            // Compute desired velocity using admittance dynamics
            Eigen::Vector3d desiredVel = generateMotionDynamics(
                currentPosition,
                m_currentVelocity,
                admittanceForce,
                controlTimestep);


            // Integrate velocity from previous commanded target (not current measured position)
            Eigen::Vector3d previousTarget(target_pose[0], target_pose[1], target_pose[2]);
            Eigen::Vector3d nextPosition = previousTarget + desiredVel * controlTimestep;

            target_pose[0] = nextPosition.x();
            target_pose[1] = nextPosition.y();
            target_pose[2] = nextPosition.z();

            // Keep orientation unchanged (qw, qx, qy, qz)
            target_pose[3] = initial_pose[3];
            target_pose[4] = initial_pose[4];
            target_pose[5] = initial_pose[5];
            target_pose[6] = initial_pose[6];

            // Send command to robot with high limits to avoid built-in interference
            robot_->SendCartesianMotionForce(
                target_pose,
                {}, // no extra force
                {}, // no extra torque
                m_maxLinearVel,
                m_maxAngularVel,
                m_maxLinearAcc,
                m_maxAngularAcc);

            // Update commanded values for logging
            setCommandedTcpPose(target_pose);
            setCommandedTcpVel({desiredVel.x(), desiredVel.y(), desiredVel.z(), 0.0, 0.0, 0.0});

            // Measure iteration time (before sleep)
            auto iteration_end = std::chrono::steady_clock::now();
            double iteration_time_ms = std::chrono::duration<double, std::milli>(iteration_end - now).count();

            sum_loop_time += iteration_time_ms;
            if (iteration_time_ms > max_loop_time) {
                max_loop_time = iteration_time_ms;
            }
            loop_count++;

            // wait until next cycle
            std::this_thread::sleep_until(next_tick);
        }

        // Print timing statistics
        double avg_loop_time = (loop_count > 0) ? (sum_loop_time / loop_count) : 0.0;
        std::cout << "[AdmittanceTest] Control loop completed\n";
        std::cout << "[AdmittanceTest] Loop timing statistics:\n";
        std::cout << "  Total iterations: " << loop_count << "\n";
        std::cout << "  Average iteration time: " << avg_loop_time << " ms\n";
        std::cout << "  Max iteration time: " << max_loop_time << " ms\n";
        std::cout << "  Target interval: " << streamIntervalMs << " ms\n";
    }
    catch (const std::exception& e) {
        std::cerr << "[AdmittanceTest] Error in control loop: " << e.what() << "\n";
        throw;
    }

#ifdef _WIN32
    timeEndPeriod(1);
#endif
}

// -------------------- performTest -------------------- //
void AdmittanceTest::performTest()
{
    try {
        std::cout << "\n========================================\n";
        std::cout << "  ADMITTANCE TEST - Real Force Feedback\n";
        std::cout << "========================================\n\n";

        std::vector<LogField> fieldsToLog = {
            LogField::EXT_WRENCH_TCP,   // External force/torque in TCP frame (6 values)
            LogField::TCP_POSE,         // Measured TCP pose (7 values)
            LogField::TCP_VEL,          // Measured TCP velocity (6 values)
            LogField::CMD_TCP_POSE,     // Commanded TCP pose (7 values)
            LogField::CMD_TCP_VEL       // Commanded TCP velocity (6 values)
        };

        std::vector<uint16_t> intervals = { 5, 25, 50, 75, 100};

        auto runPhase = [&](uint16_t intervalMs) -> bool {
            if (stopRequested_) return false;
            
            // Zero FT sensor to eliminate bias from tool weight
            std::cout << "[AdmittanceTest] Zeroing force/torque sensor...\n";
            ZeroFTSensor();
            if (stopRequested_) return false;

            // Move to starting pose before each test
            moveToStartingPose();
            if (stopRequested_) return false;

            // Wait for sensor to settle
            std::cout << "[AdmittanceTest] Waiting 3 seconds for sensor settling...\n";
            std::this_thread::sleep_for(std::chrono::seconds(3));
            if (stopRequested_) return false;

            // Start logging
            std::string filename = "admittance_test_interval_" + std::to_string(intervalMs) + "ms.csv";
            std::cout << "[AdmittanceTest] Starting data logging: " << filename << "\n";
            startLogging(filename, fieldsToLog, 1);  // Always log at 1ms interval

            //
            robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
            robot_->SetForceControlAxis({ false , false , false , false ,false ,false });

            // Run admittance control loop
            try {
                admittanceControlLoop(intervalMs);
            }
            catch (...) {
                stopLogging();
                throw;
            }

            // Stop logging
            stopLogging();
            std::cout << "[AdmittanceTest] Data logging stopped\n";

            std::this_thread::sleep_for(std::chrono::seconds(5));
            return !stopRequested_;
        };

        // Run tests for each interval
        for (auto interval : intervals) {
            std::cout << "\n========================================\n";
            std::cout << "  Testing at " << interval << " ms interval (" << (1000.0 / interval) << " Hz)\n";
            std::cout << "========================================\n\n";

            if (!runPhase(interval)) return;
        }

        std::cout << "\n========================================\n";
        std::cout << "  ALL ADMITTANCE TESTS COMPLETED\n";
        std::cout << "========================================\n\n";
    }
    catch (const std::exception& e) {
        std::cerr << "[AdmittanceTest] performTest error: " << e.what() << "\n";
    }
}
