#include "admittance_c_api.h"
#include "core/simple_logger.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <limits>
#include <cstdint>
#include <memory>

#ifdef _WIN32
  #include <windows.h>
  #include <mmsystem.h>
#endif

// -------------------- Global variable definitions -------------------- //
double g_switching_gain = 100.0;
Eigen::Vector3d g_high_end = Eigen::Vector3d::Zero();
Eigen::Vector3d g_low_end  = Eigen::Vector3d::Zero();
Eigen::Vector3d g_high_end_torque = Eigen::Vector3d::Zero();
Eigen::Vector3d g_low_end_torque  = Eigen::Vector3d::Zero();

// Global logger instance
static std::unique_ptr<SimpleLogger> g_logger = nullptr;

// -------------------- Helper functions -------------------- //

inline double mmToM(double mm)
{
    return mm * 0.001;
}

inline double mToMm(double m) {
    return m * 1000.0;
}

inline double degToRad(double deg) {
    return deg * (M_PI / 180.0);
}

inline double radToDeg(double rad) {
    return rad * (180.0 / M_PI);
}

inline Eigen::Vector3d applySmoothDeadZone3(const Eigen::Vector3d& input_s,const Eigen::Vector3d& low_end,const Eigen::Vector3d& high_end,double sw_gn)
{
    Eigen::Vector3d output;
    for (int i = 0; i < 3; ++i)
    {
        const double active_pve = 0.5 * (std::tanh(sw_gn * ( input_s[i] - high_end[i])) + 1.0);
        const double active_nve = 0.5 * (std::tanh(sw_gn * (-input_s[i] +  low_end[i])) + 1.0);

        output[i] = active_nve * (input_s[i] - low_end[i]) + active_pve * (input_s[i] - high_end[i]);
    }
    return output;
}

inline Eigen::Vector3d applySmoothSaturation(const Eigen::Vector3d& input, double max_limit)
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

    if (quat_error[0] < 0.0)
        quat_error = -quat_error;

    return 2.0 * quat_error.tail<3>();
}

inline Eigen::Vector4d update_orientation_dynamics(const Eigen::Vector4d& q_in, const Eigen::Vector3d& omega) {
    const Eigen::Vector4d q = quat_normalize(q_in);
    const double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
    const double wx = omega(0), wy = omega(1), wz = omega(2);
    Eigen::Vector4d q_dot;
    q_dot << -qx*wx - qy*wy - qz*wz,
              qw*wx + qy*wz - qz*wy,
              qw*wy - qx*wz + qz*wx,
              qw*wz + qx*wy - qy*wx;

    return 0.5 * q_dot;
}

inline Eigen::Vector4d integrate_orientation_dynamics(double dt, const Eigen::Vector4d& q, const Eigen::Vector3d& omega, bool use_RK4 = false) {
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

inline std::array<double, 3> quat_to_eulerZYX(const std::array<double, 4>& quat)
{
    // Form quaternion
    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);

    // The returned array is in [z,y,x] order
    auto euler_ZYX = q.toRotationMatrix().eulerAngles(2, 1, 0);

    // Convert to general [x,y,z] order
    return (std::array<double, 3> {euler_ZYX[2], euler_ZYX[1], euler_ZYX[0]});
}

// Helper to convert pointer to std::array
inline std::array<double, 6> ptrToArray6(const double* ptr) {
    return {ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]};
}

std::array<double, 7> convertPose_mmDeg_to_mQuat(const std::array<double, 6>& pose_mm_deg)
{
    // Convert position from mm to meters
    double x_m = mmToM(pose_mm_deg[0]); //
    double y_m = mmToM(pose_mm_deg[1]); //
    double z_m = mmToM(pose_mm_deg[2]); //

    // Convert Euler angles from degrees to radians
    double yaw_rad   = degToRad(pose_mm_deg[3]); //
    double pitch_rad = degToRad(pose_mm_deg[4]); //
    double roll_rad  = degToRad(pose_mm_deg[5]); //

    // Convert to quaternion using your existing helper
    Eigen::Quaterniond quat = eulerZYX_to_quat(yaw_rad, pitch_rad, roll_rad); //

    // Return as {x, y, z, qw, qx, qy, qz}
    // Note: Eigen stores internal coeffs as (x, y, z, w),
    // but quat.w() etc. are the explicit ways to get the components.
    return {
        x_m, y_m, z_m,
        quat.w(), quat.x(), quat.y(), quat.z()
    }; //
}

void admittance_init(
    const double* high_end,        // Array of 3: x, y, z thresholds
    const double* high_end_torque, // Array of 3: tx, ty, tz thresholds
    double switching_gain,
    const char* log_filename       // Log file path (NULL to disable logging)
) {
    g_switching_gain = switching_gain;

    for(int i=0; i<3; ++i) {
        g_high_end[i] = high_end[i];
        g_low_end[i]  = -high_end[i]; // Assuming symmetric deadzone

        g_high_end_torque[i] = high_end_torque[i];
        g_low_end_torque[i]  = -high_end_torque[i];
    }

    // Initialize logger if filename provided
    if (log_filename != nullptr && log_filename[0] != '\0') {
        try {
            g_logger = std::make_unique<SimpleLogger>(log_filename);
            g_logger->header(
                "cur_pos_x(m),cur_pos_y(m),cur_pos_z(m),"                         // currentPosition (3)
                "pos_err_x(m),pos_err_y(m),pos_err_z(m),"                         // pos_error (3)
                "cur_ori_qw,cur_ori_qx,cur_ori_qy,cur_ori_qz,"                    // currentOrientation (4)
                "ori_err_x(rad),ori_err_y(rad),ori_err_z(rad),"                   // ori_error (3)
                "meas_force_x(N),meas_force_y(N),meas_force_z(N),"                // measuredForce (3)
                "adm_force_x(N),adm_force_y(N),adm_force_z(N),"                   // admittanceForceInput (3)
                "meas_torque_x(Nm),meas_torque_y(Nm),meas_torque_z(Nm),"          // measuredTorque (3)
                "adm_torque_x(Nm),adm_torque_y(Nm),adm_torque_z(Nm),"             // admittanceTorqueInput (3)
                "cur_lin_vel_x(m/s),cur_lin_vel_y(m/s),cur_lin_vel_z(m/s),"       // currentLinVelocity (3)
                "cur_ang_vel_x(rad/s),cur_ang_vel_y(rad/s),cur_ang_vel_z(rad/s)," // currentAngVelocity (3)
                "des_vel_x(m/s),des_vel_y(m/s),des_vel_z(m/s),"                   // desired linear velocity (3)
                "des_vel_wx(rad/s),des_vel_wy(rad/s),des_vel_wz(rad/s),"          // desired angular velocity (3)
                "des_pos_x(m),des_pos_y(m),des_pos_z(m),"                         // desired position (3)
                "des_ori_qw,des_ori_qx,des_ori_qy,des_ori_qz"                     // desired orientation (4)
            );
        } catch (const std::exception& e) {
            // Failed to create logger - continue without logging
            g_logger = nullptr;
        }
    }
}

void admittance_deinit() {
    if (g_logger) {
        g_logger->close();
        g_logger.reset();
    }
}

//main call
void generateFullMotionDynamics(
    const double* cur_position_mm_deg,
    const double* desired_position_mm_deg,
    const double* curr_vel,
    const double* curr_force_torque,
    double* desired_vel,
    double dt_ms,
    double pos_m,
    double pos_k,
    double pos_d,
    double ori_m,
    double ori_k,
    double ori_d,
    int32_t is_second_order,
    int32_t is_admittance,
    double MAX_LIN_VEL,
    double MAX_ANG_VEL
) {
    const double dt = dt_ms * 0.001;

    // Convert poses from pointer to array, then to m/quaternion
    // BUG FIX: Declare desired_pose_m_quat BEFORE using it
    auto desired_pose_m_quat = convertPose_mmDeg_to_mQuat(ptrToArray6(desired_position_mm_deg));
    auto curr_pose_m_quat = convertPose_mmDeg_to_mQuat(ptrToArray6(cur_position_mm_deg));

    // Get the position error
    Eigen::Vector3d curr_pos(curr_pose_m_quat[0], curr_pose_m_quat[1], curr_pose_m_quat[2]);
    Eigen::Vector3d des_pos(desired_pose_m_quat[0], desired_pose_m_quat[1], desired_pose_m_quat[2]);
    Eigen::Vector3d error_pos = curr_pos - des_pos;

    // Get the orientation error
    Eigen::Vector4d curr_ori(curr_pose_m_quat[3], curr_pose_m_quat[4], curr_pose_m_quat[5], curr_pose_m_quat[6]);
    Eigen::Vector4d des_ori(desired_pose_m_quat[3], desired_pose_m_quat[4], desired_pose_m_quat[5], desired_pose_m_quat[6]);
    Eigen::Vector3d error_ori = ori_error_from_quat(curr_ori, des_ori);

    // Get the velocities in eigen vector
    Eigen::Vector3d curr_lin_vel(mmToM(curr_vel[0]), mmToM(curr_vel[1]), mmToM(curr_vel[2]));
    Eigen::Vector3d curr_ang_vel(degToRad(curr_vel[3]), degToRad(curr_vel[4]), degToRad(curr_vel[5]));

    // Apply deadzone to force
    // BUG FIX: Use g_low_end, g_high_end, g_switching_gain (not m_xxx)
    Eigen::Vector3d curr_force(curr_force_torque[0], curr_force_torque[1], curr_force_torque[2]);
    Eigen::Vector3d admittance_force = applySmoothDeadZone3(curr_force, g_low_end, g_high_end, g_switching_gain);

    // Apply deadzone to torque
    // BUG FIX: Use g_low_end_torque, g_high_end_torque (not m_xxx)
    Eigen::Vector3d curr_torque(curr_force_torque[3], curr_force_torque[4], curr_force_torque[5]);
    Eigen::Vector3d admittance_torque = applySmoothDeadZone3(curr_torque, g_low_end_torque, g_high_end_torque, g_switching_gain);

    // Build diagonal pos mass/stiffness/damping matrices (Matrix3d needed for .inverse())
    Eigen::Matrix3d pos_k_vec = Eigen::Vector3d(pos_k, pos_k, pos_k).asDiagonal();
    Eigen::Matrix3d pos_d_vec = Eigen::Vector3d(pos_d, pos_d, pos_d).asDiagonal();
    Eigen::Matrix3d pos_invM_vec = Eigen::Vector3d(1.0/pos_m, 1.0/pos_m, 1.0/pos_m).asDiagonal();

    // Build diagonal ori mass/stiffness/damping matrices (Matrix3d needed for .inverse())
    Eigen::Matrix3d ori_k_vec = Eigen::Vector3d(ori_k, ori_k, ori_k).asDiagonal();
    Eigen::Matrix3d ori_d_vec = Eigen::Vector3d(ori_d, ori_d, ori_d).asDiagonal();
    Eigen::Matrix3d ori_invM_vec = Eigen::Vector3d(1.0/ori_m, 1.0/ori_m, 1.0/ori_m).asDiagonal();

    // Is admittance
    double transmit_wrench = is_admittance ? 1.0 : 0.0;

    // Initialisations
    Eigen::Vector3d des_velocity_lin     = Eigen::Vector3d::Zero();
    Eigen::Vector3d des_acceleration_lin = Eigen::Vector3d::Zero();
    Eigen::Vector3d des_velocity_ang     = Eigen::Vector3d::Zero();
    Eigen::Vector3d des_acceleration_ang = Eigen::Vector3d::Zero();

    if (is_second_order) {
        // Compute stiffness force and moment
        Eigen::Vector3d stiffness_force  = pos_k_vec * error_pos;
        Eigen::Vector3d stiffness_torque = ori_k_vec * error_ori;

        // Compute equivalent desired velocity (when looking at the 2nd order admittance as a velocity following dynamics)
        Eigen::Vector3d equival_des_lin_vel = -pos_d_vec.inverse() * (stiffness_force  - transmit_wrench * admittance_force);
        Eigen::Vector3d equival_des_ang_vel = -ori_d_vec.inverse() * (stiffness_torque - transmit_wrench * admittance_torque);

        // Apply velocity limits to yield acceleration that is consistent with the velocity limits
        Eigen::Vector3d sat_equival_des_lin_vel = applySmoothSaturation(equival_des_lin_vel, MAX_LIN_VEL);
        Eigen::Vector3d sat_equival_des_ang_vel = applySmoothSaturation(equival_des_ang_vel, MAX_ANG_VEL);

        // Compute the equivalent damping force and moment
        Eigen::Vector3d equival_damping_force  = pos_d_vec * (curr_lin_vel - sat_equival_des_lin_vel);
        Eigen::Vector3d equival_damping_torque = ori_d_vec * (curr_ang_vel - sat_equival_des_ang_vel);

        // Compute desired acceleration
        des_acceleration_lin = -pos_invM_vec * equival_damping_force;
        des_acceleration_ang = -ori_invM_vec * equival_damping_torque;

        // Compute desired velocity
        des_velocity_lin = curr_lin_vel + des_acceleration_lin * dt;
        des_velocity_ang = curr_ang_vel + des_acceleration_ang * dt;
    } else {
        // Get the 1st order gains from the 2nd order gains
        Eigen::Matrix3d InvM_posK = pos_invM_vec *  pos_k_vec;
        Eigen::Matrix3d InvM_oriK = ori_invM_vec *  ori_k_vec;

        Eigen::Vector3d K_pos_diag = InvM_posK.diagonal().cwiseSqrt();
        Eigen::Matrix3d K_pos_1st_order = K_pos_diag.asDiagonal();
        Eigen::Vector3d K_ori_diag = InvM_oriK.diagonal().cwiseSqrt();
        Eigen::Matrix3d K_ori_1st_order = K_ori_diag.asDiagonal();

        // Compute desired velocity
        des_velocity_lin = -K_pos_1st_order * (error_pos - transmit_wrench * (pos_k_vec.inverse() * admittance_force));
        des_velocity_ang = -K_ori_1st_order * (error_ori - transmit_wrench * (ori_k_vec.inverse() * admittance_torque));
    }

    // Apply velocity limits
    Eigen::Vector3d des_sat_vel_lin = applySmoothSaturation(des_velocity_lin, MAX_LIN_VEL);
    Eigen::Vector3d des_sat_vel_ang = applySmoothSaturation(des_velocity_ang, MAX_ANG_VEL);

    desired_vel[0] = des_sat_vel_lin[0];
    desired_vel[1] = des_sat_vel_lin[1];
    desired_vel[2] = des_sat_vel_lin[2];
    desired_vel[3] = des_sat_vel_ang[0];
    desired_vel[4] = des_sat_vel_ang[1];
    desired_vel[5] = des_sat_vel_ang[2];

    // Log data if logger is initialized
    if (g_logger && g_logger->isOpen()) {
        *g_logger << curr_pos                   // current position (3)
                  << error_pos                  // pos_error (3)
                  << curr_ori                   // current orientation (4)
                  << error_ori                  // ori_error (3)
                  << curr_force                 // measuredForce (3)
                  << admittance_force           // admittanceForceInput (3)
                  << curr_torque                // measuredTorque (3)
                  << admittance_torque          // admittanceTorqueInput (3)
                  << curr_lin_vel               // currentLinVelocity (3)
                  << curr_ang_vel               // currentAngVelocity (3)
                  << des_sat_vel_lin            // desired linear velocity (3)
                  << des_sat_vel_ang            // desired angular velocity (3)
                  << des_pos                    // desired position (3)
                  << des_ori                    // desired orientation (4)
                  << ::endl;
    }
}