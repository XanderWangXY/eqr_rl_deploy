#ifndef EHR_HARDWARE_DEF_H_
#define EHR_HARDWARE_DEF_H_

struct ehr_body_state
{
  double real_time;

  // Base frame posture: [translation (x, y, z), quaternion (w, x, y, z)]
  double q_base[7];

  // Base frame posture: [translation (x, y, z), Euler angles (roll, pitch, yaw)]
  double q2_base[6];

  // Base frame velocity: [linear (x, y, z), angular (roll, pitch, yaw)]
  double v_base[6];

  // Base frame acceleration: [linear (x, y, z), angular (roll, pitch, yaw)]
  double a_base[6];

  // Base frame magnetometer
  double magnetometer[3];

  // Joint information
  // The order of all joints is: [FL_HIP_X, FL_HIP_Y, FL_KNEE, RF_HIP_X, RF_HIP_Y, RF_KNEE, LH_HIP_X, LH_HIP_Y, LH_KNEE, RH_HIP_X, RH_HIP_Y, RH_KNEE]
  int joint_num;
  double *q_joints;           // Joint positions
  double *v_joints;           // Joint velocities
  double *f_joints;           // Joint force/torques
  double *gear_ratios;        // Gear ratios

  // Desired joint states
  double *q_joint_desired;    // Desired joint positions
  double *v_joint_desired;    // Desired joint velocities
  double *ff_joint_desired;   // Desired joint torques

  // Joint space gains
  double *kp_joints;          // Proportional gains
  double *kd_joints;          // Derivative gains
  double *ki_joints;          // Integral gains

  // Torso contact sensors
  int torso_contact_num;
  double *torso_cfrcs;       // Torso contact forces
  double *torso_cfrcs_state;
  
  // Contact forces for Quadruped Robot: [left front, right front, left hind, right hind]
  double lf_cfrc[6];          // Left front leg  / Left foot contact force
  double rf_cfrc[6];          // Right front leg / Right foot contact force
  double lh_cfrc[6];          // Left hind leg / Left hand contact force
  double rh_cfrc[6];          // Right hind leg / Right hand contact force

  // Force sensor states: [0: invalid, 1: valid]
  int f_cfrc_state_num;
  int h_cfrc_state_num;
  double *lf_cfrc_states;   // Left front leg / Left foot force sensor state
  double *rf_cfrc_states;   // Right front leg / Right foot force sensor state
  double *lh_cfrc_states;   // Left hind leg / Left hand force sensor state
  double *rh_cfrc_states;   // Right hind leg / Right hand force sensor state
};

#endif // EHR_HARDWARE_DEF_H_
