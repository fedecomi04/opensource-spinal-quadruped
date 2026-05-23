#ifndef SENSOR_IDS_H
#define SENSOR_IDS_H
extern int id_imu_gyro;     // 3-DOF gyro at imu_site
extern int id_imu_quat; 
extern int id_imu_angvel;   // angular‐velocity sensor
extern int id_imu_vel;
extern int id_imu_site;

extern int id_FR_hip_s_tau;
extern int id_FR_hip_s_pos;
extern int id_FR_hip_s_vel;
extern int id_FR_femur_s_tau;
extern int id_FR_femur_s_pos;
extern int id_FR_femur_s_vel;
extern int id_FR_tibia_s_tau;
extern int id_FR_tibia_s_pos;
extern int id_FR_tibia_s_vel;
extern int id_FL_hip_s_tau;
extern int id_FL_hip_s_pos;
extern int id_FL_hip_s_vel;
extern int id_FL_femur_s_tau;
extern int id_FL_femur_s_pos;
extern int id_FL_femur_s_vel;
extern int id_FL_tibia_s_tau;
extern int id_FL_tibia_s_pos;
extern int id_FL_tibia_s_vel;
extern int id_RR_hip_s_tau;
extern int id_RR_hip_s_pos;
extern int id_RR_hip_s_vel;
extern int id_RR_femur_s_tau;
extern int id_RR_femur_s_pos;
extern int id_RR_femur_s_vel;
extern int id_RR_tibia_s_tau;
extern int id_RR_tibia_s_pos;
extern int id_RR_tibia_s_vel;
extern int id_RL_hip_s_tau;
extern int id_RL_hip_s_pos;
extern int id_RL_hip_s_vel;
extern int id_RL_femur_s_tau;
extern int id_RL_femur_s_pos;
extern int id_RL_femur_s_vel;
extern int id_RL_tibia_s_tau;
extern int id_RL_tibia_s_pos;
extern int id_RL_tibia_s_vel;
extern int id_F_spine_s_tau;
extern int id_F_spine_s_pos;
extern int id_F_spine_s_vel;
extern int id_RL_spine_s_tau;
extern int id_RL_spine_s_pos;
extern int id_RL_spine_s_vel;
extern int id_RR_spine_s_tau;
extern int id_RR_spine_s_pos;
extern int id_RR_spine_s_vel;

extern int id_UJ_spine_s_tau;
extern int id_UJ_spine_s_pos;
extern int id_UJ_spine_s_vel;

void init_sensor_ids(const mjModel* m);

#endif