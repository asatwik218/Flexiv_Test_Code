#include <Eigen/Dense>

// C-compatible header for the admittance dynamics DLL
#ifdef __cplusplus
extern "C" {
#endif

#ifdef ADMITTANCE_DLL_EXPORTS
  #define ADMITTANCE_API __declspec(dllexport)
#else
  #define ADMITTANCE_API __declspec(dllimport)
#endif

// Global variables (declared extern, defined in .c file)
extern double g_switching_gain;
extern Eigen::Vector3d g_high_end;
extern Eigen::Vector3d g_low_end;
extern Eigen::Vector3d g_high_end_torque;
extern Eigen::Vector3d g_low_end_torque;

// Initialize admittance controller and logger
// log_filename: path for CSV log file (e.g., "logs/admittance.csv")
ADMITTANCE_API void admittance_init(
    const double* high_end,        // High force threshold [N] (array of 3)
    const double* high_end_torque, // High torque threshold [Nm] (array of 3)
    double switching_gain,
    const char* log_filename       // Log file path (NULL to disable logging)
);

// Deinitialize and close logger
ADMITTANCE_API void admittance_deinit();

// Generate full 6-DOF motion dynamics
ADMITTANCE_API void generateFullMotionDynamics(
    const double* cur_position_mm_deg,      // Current pose [x,y,z,rx,ry,rz] in mm and degrees (array of 6)
    const double* desired_position_mm_deg,  // Desired pose [x,y,z,rx,ry,rz] in mm and degrees (array of 6)
    const double* curr_vel,                 // Current velocity [vx,vy,vz,wx,wy,wz] in mm/s and deg/s (array of 6)
    const double* curr_force_torque,        // Current force/torque [fx,fy,fz,tx,ty,tz] in N and Nm (array of 6)
    double* desired_vel,                    // Output: desired velocity [vx,vy,vz,wx,wy,wz] in m/s and rad/s (array of 6)
    double dt_ms = 20.0,
    double pos_m = 1.0,
    double pos_k = 50.0,
    double pos_d = 14.14,
    double ori_m = 0.05,
    double ori_k = 40.0,
    double ori_d = 4.0,
    int32_t is_second_order = 1,
    int32_t is_admittance = 1,
    double MAX_LIN_VEL = 0.1,
    double MAX_ANG_VEL = 10.0
);

#ifdef __cplusplus
}
#endif
