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
    double ori_m,
    double ori_k,
    const std::array<double, 3>& high_end,
    const std::array<double, 3>& high_end_torque
): FlexivRobotTest("AdmittanceTest", robotSn),
      m_startingPose_mm_deg(startingPose_mm_deg),
      m_endPose_mm_deg(endPose_mm_deg),
      m_isAdmittance(isAdmittance),
      m_testDuration(testDuration),
      m_pos_m(pos_m),
      m_pos_k(pos_k),
      m_ori_m(ori_m),
      m_ori_k(ori_k)
{
    // Compute damping (critical damping)
    m_pos_d = 2.0 * std::sqrt(m_pos_m * m_pos_k);
    m_ori_d = 2.0 * std::sqrt(m_ori_m * m_ori_k);

    // Build diagonal mass/stiffness/damping matrices (direct initialization)
    m_K = Eigen::Vector3d(m_pos_k, m_pos_k, m_pos_k).asDiagonal();
    m_D = Eigen::Vector3d(m_pos_d, m_pos_d, m_pos_d).asDiagonal();
    m_InvM = Eigen::Vector3d(1.0/m_pos_m, 1.0/m_pos_m, 1.0/m_pos_m).asDiagonal();

    m_K_orientation = Eigen::Vector3d(m_ori_k, m_ori_k, m_ori_k).asDiagonal();
    m_D_orientation = Eigen::Vector3d(m_ori_d, m_ori_d, m_ori_d).asDiagonal();
    m_InvM_orientation = Eigen::Vector3d(1.0/m_ori_m, 1.0/m_ori_m, 1.0/m_ori_m).asDiagonal();

    // pack force thresholds
    m_high_end = Eigen::Vector3d(high_end[0], high_end[1], high_end[2]);
    m_low_end = -m_high_end;
    // pack torque thresholds
    m_high_end_torque = Eigen::Vector3d(high_end_torque[0], high_end_torque[1], high_end_torque[2]);
    m_low_end_torque = -m_high_end_torque;

    std::array<double, 7> pose_m_quat = convertPose_mmDeg_to_mQuat(endPose_mm_deg);
    m_desiredPosition = Eigen::Vector3d( pose_m_quat[0],pose_m_quat[1],pose_m_quat[2] );
    m_desiredOrientation = Eigen::Vector4d( pose_m_quat[3],pose_m_quat[4],pose_m_quat[5],pose_m_quat[6] );
   
}

// -------------------- Helper functions -------------------- //
inline Eigen::Vector3d AdmittanceTest::applySmoothDeadZone3(
    const Eigen::Vector3d& input_s,
    const Eigen::Vector3d& low_end,
    const Eigen::Vector3d& high_end,
    double sw_gn)
{
    // Apply smooth dead-zone to a 3x1 signal vector.
    // input_s : signal (3x1)
    // low_end : low threshold per component (3x1)
    // high_end: high threshold per component (3x1)
    // sw_gn   : switching gain (default 100.0), higher -> sharper transition

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

inline Eigen::Vector4d quat_conj(const Eigen::Vector4d& q) {
    Eigen::Vector4d qc;
    qc << q(0), -q(1), -q(2), -q(3);
    return qc;
}

inline Eigen::Vector4d quat_mul(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
    const double w1 = q1(0), x1 = q1(1), y1 = q1(2), z1 = q1(3);
    const double w2 = q2(0), x2 = q2(1), y2 = q2(2), z2 = q2(3);

    Eigen::Vector4d out;
    out <<  w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2;
    return out;
}

inline Eigen::Vector4d quat_normalize(const Eigen::Vector4d& q, double eps = 1e-12) {
    const double n = q.norm(); // sqrt(sum(q_i^2))
    if (n < eps) {
        // throw std::runtime_error("quat_normalize: norm is too small (near-zero quaternion).");
        return Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    }
    else{
        return q / n;
    }
}

inline Eigen::Vector3d ori_error_from_quat(const Eigen::Vector4d& cur_orientation, const Eigen::Vector4d& des_orientation) {
    Eigen::Vector4d quat_error = quat_mul(quat_conj(des_orientation), cur_orientation);
    quat_error = quat_normalize(quat_error);
    if (quat_error[0] < 0.0) {
        quat_error = -quat_error;
    }
    return 2.0 * quat_error.tail<3>();
}

inline Eigen::Vector4d update_orientation_dynamics(const Eigen::Vector4d& q_in, const Eigen::Vector3d& omega) {
    const Eigen::Vector4d q = quat_normalize(q_in);

    // Quaternion convention: q = [w, x, y, z] where q(0)=w, q(1)=x, q(2)=y, q(3)=z
    const double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
    const double wx = omega(0), wy = omega(1), wz = omega(2);

    // Quaternion derivative: q_dot = 0.5 * q ⊗ [0; omega]
    // For q = [w, x, y, z] and omega = [wx, wy, wz]:
    // q_dot = 0.5 * [-x*wx - y*wy - z*wz,
    //                 w*wx + y*wz - z*wy,
    //                 w*wy - x*wz + z*wx,
    //                 w*wz + x*wy - y*wx]
    Eigen::Vector4d q_dot;
    q_dot << -qx*wx - qy*wy - qz*wz,
              qw*wx + qy*wz - qz*wy,
              qw*wy - qx*wz + qz*wx,
              qw*wz + qx*wy - qy*wx;

    return 0.5 * q_dot;
}

/**
 * Integrate quaternion dynamics.
 * dt: timestep
 * q:  (4x1) quaternion [w,x,y,z] (scalar-first convention)
 * omega: (3x1) angular velocity (assumed constant over dt, matching Python)
 * use_RK4: if false -> Euler; if true -> RK4
 * returns: (4x1) normalized quaternion [w,x,y,z]
 */
inline Eigen::Vector4d integrate_orientation_dynamics(  double dt,
                                                        const Eigen::Vector4d& q,
                                                        const Eigen::Vector3d& omega,
                                                        bool use_RK4 = false) {
    Eigen::Vector4d q_out;

    if (!use_RK4) {
        q_out = q + update_orientation_dynamics(q, omega) * dt;
    } else {
        const Eigen::Vector4d k1 = update_orientation_dynamics(q, omega);
        const Eigen::Vector4d k2 = update_orientation_dynamics(q + 0.5 * k1 * dt, omega);
        const Eigen::Vector4d k3 = update_orientation_dynamics(q + 0.5 * k2 * dt, omega);
        const Eigen::Vector4d k4 = update_orientation_dynamics(q +       k3 * dt, omega);

        // q_out = q + (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt;
        q_out = q + 0.166666666666 * (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt;
    }

    return quat_normalize(q_out);
}

inline Eigen::Quaterniond eulerZYX_to_quat(double yaw, double pitch, double roll)
{
    Eigen::AngleAxisd yawAA  (yaw,   Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAA(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAA (roll,  Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = yawAA * pitchAA * rollAA;  // Rz * Ry * Rx
    q.normalize();
    return q;
}

Eigen::Vector3d AdmittanceTest::generateMotionDynamics(
    const Eigen::Vector3d& cur_position,
    const Eigen::Vector3d& cur_lin_vel_ee,
    const Eigen::Vector3d& adm_force_ee,
    double dt,
    bool is_second_order) const
{
    // Use member variables directly
    const Eigen::Vector3d pos_error = cur_position - m_desiredPosition;
    const double transmit_wrench = m_isAdmittance ? 1.0 : 0.0;
    
    Eigen::Vector3d des_velocity_lin = Eigen::Vector3d::Zero();
    Eigen::Vector3d des_acceleration_lin = Eigen::Vector3d::Zero();

    if (is_second_order) {
        // Second-order dynamics (mass-spring-damper) - direct matrix access, no map lookup
        const Eigen::Vector3d stiffness_force = m_K * pos_error;
        const Eigen::Vector3d equival_des_lin_vel = -m_D.inverse() * (stiffness_force - transmit_wrench * adm_force_ee);
        const Eigen::Vector3d sat_equival_des_lin_vel = applySmoothSaturation(equival_des_lin_vel, m_maxLinearVel);
        const Eigen::Vector3d equival_damping_force = m_D * (cur_lin_vel_ee - sat_equival_des_lin_vel);
        const Eigen::Vector3d des_acceleration_lin = -m_InvM * equival_damping_force;
        des_velocity_lin = cur_lin_vel_ee + des_acceleration_lin * dt;
    }else {
        // Get the 1st order gains from the 2nd order gains
        Eigen::Matrix3d InvM_posK = m_InvM * m_K;
        Eigen::Vector3d K_pos_diag = InvM_posK.diagonal().cwiseSqrt();
        Eigen::Matrix3d K_pos_1st_order = K_pos_diag.asDiagonal();
        // Compute desired velocity
        des_velocity_lin = -K_pos_1st_order * (pos_error - transmit_wrench * (m_K.inverse() * adm_force_ee));
    }
    return applySmoothSaturation(des_velocity_lin, m_maxLinearVel);
}

Eigen::Matrix<double, 6, 1> AdmittanceTest::generateFullMotionDynamics(
        const Eigen::Vector3d& cur_position,
        const Eigen::Vector4d& cur_orientation,
        const Eigen::Vector3d& cur_lin_vel_ee,
        const Eigen::Vector3d& cur_ang_vel_ee,
        const Eigen::Vector3d& adm_force_ee,
        const Eigen::Vector3d& adm_torque_ee,
        double dt,
        bool is_second_order,
        double MAX_LIN_VEL,
        double MAX_ANG_VEL
    ) const {

    Eigen::Vector3d pos_error = cur_position - m_desiredPosition;
    Eigen::Vector3d ori_error = ori_error_from_quat(cur_orientation, m_desiredOrientation);

    // Store errors for logging
    m_lastPosError = pos_error;
    m_lastOriError = ori_error;

    double transmit_wrench = m_isAdmittance ? 1.0 : 0.0;

    Eigen::Vector3d des_velocity_lin     = Eigen::Vector3d::Zero();
    Eigen::Vector3d des_acceleration_lin = Eigen::Vector3d::Zero();
    Eigen::Vector3d des_velocity_ang     = Eigen::Vector3d::Zero();
    Eigen::Vector3d des_acceleration_ang = Eigen::Vector3d::Zero();

    if (is_second_order) {
        // Compute stiffness force and moment
        Eigen::Vector3d stiffness_force  = m_K * pos_error;
        Eigen::Vector3d stiffness_torque = m_K_orientation * ori_error;
        // Compute equivalent desired velocity (when looking at the 2nd order admittance as a velocity following dynamics)
        Eigen::Vector3d equival_des_lin_vel = -m_D.inverse() * (stiffness_force  - transmit_wrench * adm_force_ee);
        Eigen::Vector3d equival_des_ang_vel = -m_D_orientation.inverse() * (stiffness_torque - transmit_wrench * adm_torque_ee);
        // Apply velocity limits to yield acceleration that is consistent with the velocity limits
        Eigen::Vector3d sat_equival_des_lin_vel = applySmoothSaturation(equival_des_lin_vel, MAX_LIN_VEL);
        Eigen::Vector3d sat_equival_des_ang_vel = applySmoothSaturation(equival_des_ang_vel, MAX_ANG_VEL);
        // Compute the equivalent damping force and moment
        Eigen::Vector3d equival_damping_force  = m_D * (cur_lin_vel_ee - sat_equival_des_lin_vel);
        Eigen::Vector3d equival_damping_torque = m_D_orientation * (cur_ang_vel_ee - sat_equival_des_ang_vel);
        // Compute desired acceleration
        des_acceleration_lin = -m_InvM * equival_damping_force;
        des_acceleration_ang = -m_InvM_orientation * equival_damping_torque;
        // Compute desired velocity
        des_velocity_lin = cur_lin_vel_ee + des_acceleration_lin * dt;
        des_velocity_ang = cur_ang_vel_ee + des_acceleration_ang * dt;
    } else {
        // Get the 1st order gains from the 2nd order gains
        Eigen::Matrix3d InvM_posK = m_InvM *  m_K;
        Eigen::Matrix3d InvM_oriK = m_InvM_orientation *  m_K_orientation;

        Eigen::Vector3d K_pos_diag = InvM_posK.diagonal().cwiseSqrt();
        Eigen::Matrix3d K_pos_1st_order = K_pos_diag.asDiagonal();
        Eigen::Vector3d K_ori_diag = InvM_oriK.diagonal().cwiseSqrt();
        Eigen::Matrix3d K_ori_1st_order = K_ori_diag.asDiagonal();
        // Compute desired velocity
        des_velocity_lin = -K_pos_1st_order * (pos_error - transmit_wrench * (m_K.inverse() * adm_force_ee));
        des_velocity_ang = -K_ori_1st_order * (ori_error - transmit_wrench * (m_K_orientation.inverse() * adm_torque_ee));
    }

    // Apply velocity limits
    Eigen::Vector3d des_sat_vel_lin = applySmoothSaturation(des_velocity_lin, MAX_LIN_VEL);
    Eigen::Vector3d des_sat_vel_ang = applySmoothSaturation(des_velocity_ang, MAX_ANG_VEL);

    Eigen::Matrix<double, 6, 1> des_sat_vel;
    des_sat_vel << des_sat_vel_lin, des_sat_vel_ang;

    return des_sat_vel;
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

        // Initialize pose
        std::array<double, 7> target_pose = states.tcp_pose;

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
            Eigen::Vector4d currentOrientation(states.tcp_pose[3], states.tcp_pose[4], states.tcp_pose[5], states.tcp_pose[6]);

            Eigen::Vector3d currentLinVelocity(states.tcp_vel[0], states.tcp_vel[1], states.tcp_vel[2]);
            Eigen::Vector3d currentAngVelocity(states.tcp_vel[3], states.tcp_vel[4], states.tcp_vel[5]);

            Eigen::Vector3d measuredForce(-states.ext_wrench_in_tcp[0],states.ext_wrench_in_tcp[1], -states.ext_wrench_in_tcp[2]);
            Eigen::Vector3d measuredTorque(states.ext_wrench_in_tcp[3],states.ext_wrench_in_tcp[4], 10*states.ext_wrench_in_tcp[5]);

            Eigen::Vector3d admittanceForce = applySmoothDeadZone3(measuredForce, m_low_end, m_high_end, m_switchingGain);
            Eigen::Vector3d admittanceTorque = applySmoothDeadZone3(measuredTorque, m_low_end_torque, m_high_end_torque, m_switchingGain);
            
            // std::cout<< admittanceTorque[0] << "," << admittanceTorque[1] << "," << admittanceTorque[2] <<"\n";
            
            Eigen::Matrix<double,6,1> des_twist_vel = generateFullMotionDynamics(
                currentPosition,
                currentOrientation,
                currentLinVelocity,
                currentAngVelocity,
                admittanceForce,
                admittanceTorque,
                controlTimestep
            );

            Eigen::Vector3d desiredLinVel = des_twist_vel.head<3>();
            Eigen::Vector3d desiredAngVel = des_twist_vel.tail<3>();  

            // Integrate velocity from previous commanded target (not current measured position)
            Eigen::Vector3d targetPose(target_pose[0], target_pose[1], target_pose[2]);
            targetPose = targetPose + desiredLinVel * controlTimestep;

            Eigen::Vector4d targetOrientation(target_pose[3], target_pose[4], target_pose[5], target_pose[6]);
            targetOrientation = integrate_orientation_dynamics(controlTimestep, targetOrientation, desiredAngVel);

            target_pose[0] = targetPose.x();
            target_pose[1] = targetPose.y();
            target_pose[2] = targetPose.z();

            // Update orientation (qw, qx, qy, qz)
            target_pose[3] = targetOrientation[0];
            target_pose[4] = targetOrientation[1];
            target_pose[5] = targetOrientation[2];
            target_pose[6] = targetOrientation[3];

            // Send command to robot with high limits to avoid built-in interference
            robot_->SendCartesianMotionForce(
                target_pose,
                {}, // no extra force
                {}, // no extra velocity
                m_maxLinearVel,
                m_maxAngularVel,
                m_maxLinearAcc,
                m_maxAngularAcc);

            // Update commanded values and errors for logging
            setCommandedTcpPose(target_pose);
            setCommandedTcpVel({desiredLinVel.x(), desiredLinVel.y(), desiredLinVel.z(), desiredAngVel.x(), desiredAngVel.y(), desiredAngVel.z()});
            setPoseError({m_lastPosError.x(), m_lastPosError.y(), m_lastPosError.z()});
            setOriError({m_lastOriError.x(), m_lastOriError.y(), m_lastOriError.z()});

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
            LogField::CMD_TCP_VEL,      // Commanded TCP velocity (6 values)
            LogField::POSE_ERROR,       // Position error (3 values)
            LogField::ORI_ERROR         // Orientation error (3 values)
        };

        std::vector<uint16_t> intervals = {20};

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
