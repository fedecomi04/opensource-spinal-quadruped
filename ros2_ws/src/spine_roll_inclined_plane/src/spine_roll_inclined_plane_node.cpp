#include <cstdio>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <yesense_interface/msg/euler_only.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <mutex>
using namespace std::chrono_literals;

constexpr float DEG2RAD = static_cast<float>(M_PI / 180.0);
int incline_front_or_rear = 1; // =0 keep front horizontal / =1 keep rear horizontal

// ---------------------------- CONVENTIONS ----------------------------
float angle_[3] ={0}; // is an array containing the angle value of in order: pitch, roll, and yaw, in my frame: x towards
float angle_vel[3]={0};
float inclined_angle = 0;
// ------------------------------------------------------------------------------------

// ----------------------------------- Parameters [m] -----------------------------------
const double X_goal_endeffector = 0.06;

const float r_A0[3][2] = {{0,0},        {0.065,-0.065}, {0.061,0.061}};     
const float r_B0[3][2] = {{0.08,0.08},  {0.065,-0.065}, {0.061,0.061}};
const float r_C0[3][2] = {{0.08,0.08},  {0.065,-0.065}, {0,0}};

float y_o_rear = 0.176;
float z_o_rear = - 0.011;
float y_o_front = 0.188;
float z_o_front = - 0.011;

float y_P_rear = 0.232;
float z_P_rear = - 0.292;
float y_P_front = 0.246;
float z_P_front = - 0.292;

float o_initial_pos_rear[2] = {y_o_rear , z_o_rear};
float yz_initial_pos_foot_rear[2] = {y_P_rear, z_P_rear};
float o_initial_pos_front[2] = {y_o_front , z_o_front};
float yz_initial_pos_foot_front[2] = {y_P_front, z_P_front};

// ----------------------------------- Variables -----------------------------------
float theta_act[3] = {0,0,0}; // quadruped's current rear spine actuator position --> [0] : F, [1] : RR, [2] : RL

// VALID FOR ALL: leg*3 + joint (order: FR, FL, RR, RL) && spine (order: F, RR, RL)
double xyz_goal_mm[3][4];
double legs_actuator_goal_rad[3][4] = {0};
double spine_actuator_goal_rad[3] = {0};

// ----------------------------------- Variables IK -----------------------------------
float r_C_IK[3][2] = {0};
float theta_act_IK[2] = {0};
float x_c_IK[2];                // [0] : my pitch, [1] : my yaw

// ----------------------------------- Variables FK -----------------------------------
float x_c_FK[2] = {0};  // [0] : my pitch, [1] : my yaw // equivalently [0] : his roll, [1] : his pitch
float r_B_FK[3][2] = {{0}};
float r_C_FK[3][2] = {{0}};

inline void cross3(const float a[3],const float b[3],float o[3]){
    o[0]=a[1]*b[2]-a[2]*b[1];
    o[1]=a[2]*b[0]-a[0]*b[2];
    o[2]=a[0]*b[1]-a[1]*b[0];
}
inline float dot3(const float a[3],const float b[3]){
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

// -----------------------------------   Code -----------------------------------
class leg_kinematics {
public:
    void compute_inclined_foot_pos(float pitch_imu, float yz_init_foot_pos[2], float o_initial_pos[2]){
        float s = std::sin(pitch_imu),  c = std::cos(pitch_imu);
        yz_inclined_pos_foot[0] = (yz_init_foot_pos[0] * c - yz_init_foot_pos[1] * s - o_initial_pos[0]);
        yz_inclined_pos_foot[1] = (yz_init_foot_pos[0] * s + yz_init_foot_pos[1] * c - o_initial_pos[1]);

        if (incline_front_or_rear == 1){
            for (int i = 2; i < 4; i++)
            {
                xyz_goal_mm[0][i] = ((i ==2) ? -X_goal_endeffector : X_goal_endeffector);
                xyz_goal_mm[1][i] = yz_inclined_pos_foot[0];
                xyz_goal_mm[2][i] = yz_inclined_pos_foot[1];
            }
        }
        else if (incline_front_or_rear == 0){
            for (int i = 0; i < 2; i++)
            {
                xyz_goal_mm[0][i] = ((i ==1) ? -X_goal_endeffector : X_goal_endeffector);
                xyz_goal_mm[1][i] = yz_inclined_pos_foot[0];
                xyz_goal_mm[2][i] = yz_inclined_pos_foot[1];
            }
        }
    }
private: 
    float yz_inclined_pos_foot[2];
};

class IK_leg {
public:
    
    void compute_leg_IK(){ 
        // in modern robotics: x_MR = x_me, y_MR= - z_me, z_MR = y_me, from MR to me frame: + 90� x rotation of the MR frame.
        if (incline_front_or_rear == 1){ 
            for (int leg = 2; leg < 4; leg ++) {
                double px = xyz_goal_mm[0][leg];
                double py = xyz_goal_mm[1][leg] - 0.05;
                double pz = xyz_goal_mm[2][leg];

                double r2 = px*px + pz*pz;
                double phi = atan2(pz, fabs(px));
                double safe = std::max(r2 - d1*d1, 0.0);
                double alpha = atan2(sqrt(safe), d1);
                legs_actuator_goal_rad[0][leg] = fabs(phi) - fabs(alpha);
                double D = (px*px + py*py + pz*pz - d1*d1 - a2*a2 - a3*a3) / (2 * a2 * a3);
                D = std::clamp(D, -1.0, 1.0);
                legs_actuator_goal_rad[2][leg] = atan2(sqrt(1 - D*D), D); 
                double s3 = sin(legs_actuator_goal_rad[2][leg]);
                double c3 = cos(legs_actuator_goal_rad[2][leg]); 
                legs_actuator_goal_rad[1][leg] = atan2(py, sqrt(r2 - d1*d1)) - atan2(a3 * s3, a2 + a3 * c3);
                
                if (leg == 1 || leg == 2) {
                legs_actuator_goal_rad[0][leg] = - legs_actuator_goal_rad[0][leg];
                legs_actuator_goal_rad[1][leg] = - legs_actuator_goal_rad[1][leg];
                legs_actuator_goal_rad[2][leg] = - legs_actuator_goal_rad[2][leg];
                }
            }

            for (int leg = 0; leg < 2; leg ++) {
                legs_actuator_goal_rad[0][leg] = 0;
                legs_actuator_goal_rad[1][leg] = 0;
                legs_actuator_goal_rad[2][leg] = 0;
            }
        }

        else {
            for (int leg = 0; leg < 2; leg ++) {
                double px = xyz_goal_mm[0][leg];
                double py = xyz_goal_mm[1][leg] - 0.05;
                double pz = xyz_goal_mm[2][leg];

                double r2 = px*px + pz*pz;
                double phi = atan2(pz, fabs(px));
                double safe = std::max(r2 - d1*d1, 0.0);
                double alpha = atan2(sqrt(safe), d1);
                legs_actuator_goal_rad[0][leg] = fabs(phi) - fabs(alpha);
                double D = (px*px + py*py + pz*pz - d1*d1 - a2*a2 - a3*a3) / (2 * a2 * a3);
                D = std::clamp(D, -1.0, 1.0);
                legs_actuator_goal_rad[2][leg] = atan2(sqrt(1 - D*D), D); 
                double s3 = sin(legs_actuator_goal_rad[2][leg]);
                double c3 = cos(legs_actuator_goal_rad[2][leg]); 
                legs_actuator_goal_rad[1][leg] = atan2(py, sqrt(r2 - d1*d1)) - atan2(a3 * s3, a2 + a3 * c3);
                
                if (leg == 1 || leg == 2) {
                legs_actuator_goal_rad[0][leg] = - legs_actuator_goal_rad[0][leg];
                legs_actuator_goal_rad[1][leg] = - legs_actuator_goal_rad[1][leg];
                legs_actuator_goal_rad[2][leg] = - legs_actuator_goal_rad[2][leg];
                }
            }

            for (int leg = 2; leg < 4; leg ++) {
                legs_actuator_goal_rad[0][leg] = 0;
                legs_actuator_goal_rad[1][leg] = 0;
                legs_actuator_goal_rad[2][leg] = 0;
            }
        }
    }


private: 
    const double d1 = 0.06;     // x-offset (waist to hip)
    const double a2 = 0.14;     // femur length (hip to knee length)
    const double a3 = 0.141;    // tibia length (knee to end effector length)

};
class IK_spine {
public:
    void compute_spine_IK(float x_c_IK[2]) {
        float q_roll  = -x_c_IK[1]; 
        float q_pitch =  x_c_IK[0];

        compute_rC(q_roll, q_pitch);
        compute_spine_theta_actuator();
        
        spine_actuator_goal_rad[0] = 0;
        spine_actuator_goal_rad[1] = theta_act_IK[0]; 
        spine_actuator_goal_rad[2] = theta_act_IK[1];
    }

private:
    void compute_rC(float q_roll, float q_pitch) {
        float sx = std::sin(q_roll),  cx = std::cos(q_roll);
        float sy = std::sin(q_pitch), cy = std::cos(q_pitch);

        float x_rot[3][3] = {
            { cy,  sy*sx,  sy*cx },
            {  0,     cx,    -sx },
            {-sy,  cy*sx,  cy*cx }
        };

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 2; ++j)
                r_C_IK[i][j] = x_rot[i][0]*r_C0[0][j] + x_rot[i][1]*r_C0[1][j] + x_rot[i][2]*r_C0[2][j];
    }

    void compute_spine_theta_actuator() {
        for (int j = 0; j < 2; ++j) {
            float dx = r_C_IK[0][j] - r_A0[0][j];
            float dy = r_C_IK[1][j] - r_A0[1][j];
            float dz = r_C_IK[2][j] - r_A0[2][j];

            float a =  dx;
            float b = -dz;
            float d2 = dx*dx + dy*dy + dz*dz;

            float l_bar = std::sqrt(
                (r_B0[0][j]-r_A0[0][j])*(r_B0[0][j]-r_A0[0][j]) +
                (r_B0[1][j]-r_A0[1][j])*(r_B0[1][j]-r_A0[1][j]) +
                (r_B0[2][j]-r_A0[2][j])*(r_B0[2][j]-r_A0[2][j]) );

            float l_rod = std::sqrt(
                (r_C0[0][j]-r_B0[0][j])*(r_C0[0][j]-r_B0[0][j]) +
                (r_C0[1][j]-r_B0[1][j])*(r_C0[1][j]-r_B0[1][j]) +
                (r_C0[2][j]-r_B0[2][j])*(r_C0[2][j]-r_B0[2][j]) );

            float c = (l_rod*l_rod - l_bar*l_bar - d2) / (2.0f*l_bar);

            float disc = b*b*c*c - (a*a + b*b)*(c*c - a*a);
            if (disc < 0.0f) disc = 0.0f;

            float ratio = (b*c + std::sqrt(disc)) / (a*a + b*b);
            if (ratio >  1.0f) ratio =  1.0f;
            if (ratio < -1.0f) ratio = -1.0f;

            theta_act_IK[j] = std::asin(ratio);
        }
        theta_act_IK[1] = - theta_act_IK[1];
    }
};

class FK_spine
{
public:
    void compute_spine_FK(const float theta_ref_in[3])
    {   
        float theta_ref[2] = { theta_ref_in[1], theta_ref_in[2] };
        const float eps_err = 1e-6f;
        const float eps_inverse = 1e-3f;
        IK_spine ik;

        for (int it = 0; it < 20; ++it)
        {
            build_rB_rC(x_c_FK[0], x_c_FK[1], theta_ref);

            float r_bar[2][3], r_rod[2][3];
            for (int j = 0; j < 2; ++j)
                for (int i = 0; i < 3; ++i)
                {
                    r_bar[j][i] = r_B_FK[i][j] - r_A0[i][j]; // 2 lines of vector
                    r_rod[j][i] = r_C_FK[i][j] - r_B_FK[i][j]; 
                }

            float J_theta[2];
            for (int j = 0; j < 2; ++j)
            {
                float c[3]; cross3(r_bar[j], r_rod[j], c);
                J_theta[j] = (j ? c[1] :  -c[1]); // because of s_11 = (0 1 0) and s_21 = (0 -1 0) in our case
                if (std::fabs(J_theta[j]) < eps_inverse) { std::cout << "[tinyJ?]\n"; return; }
            }

            float J[2][6];
            for (int j = 0; j < 2; ++j)
            {
                float rc[3] = { r_C_FK[0][j], r_C_FK[1][j], r_C_FK[2][j] };
                float c[3];  cross3(rc, r_rod[j], c);

                J[j][0] = r_rod[j][0] / J_theta[j];
                J[j][1] = r_rod[j][1] / J_theta[j];
                J[j][2] = r_rod[j][2] / J_theta[j];
                J[j][3] = c[0]        / J_theta[j];
                J[j][4] = c[1]        / J_theta[j];
                J[j][5] = c[2]        / J_theta[j];
            }

            float ca = std::cos(x_c_FK[1]), sa = std::sin(x_c_FK[1]);

            float Jc[2][2];
            for (int j = 0; j < 2; ++j)
            {
                Jc[j][0] =  J[j][3]*ca - J[j][5]*sa;   
                Jc[j][1] =  J[j][4];                   
            }
            
            float det = Jc[0][0]*Jc[1][1] - Jc[0][1]*Jc[1][0];
            if (std::fabs(det) < eps_inverse) { std::cout << "[singJc]\n"; return; }

            float Jci[2][2] = { {  Jc[1][1]/det, -Jc[0][1]/det },
                                { -Jc[1][0]/det,  Jc[0][0]/det } };

            float g[2] = {x_c_FK[1],-x_c_FK[0]};
            ik.compute_spine_IK(g);
            //printf("FK pitch %f rad, yaw %f rad \n,", x_c_FK[1],x_c_FK[0] );
            //printf("IK(FK) RR spine %f rad, RL spine %f rad \n", theta_act_IK[0],theta_act_IK[1] );
            //printf("goal;  RR spine %f rad, RL spine %f rad \n", theta_ref[0],theta_ref[1] );
            float e[2] = { theta_act_IK[0] - theta_ref[0],
                           theta_act_IK[1] - theta_ref[1] };

            if (std::max(std::fabs(e[0]), std::fabs(e[1])) < eps_err)
            { 
                //std::cout << "[conv " << it << "]\n"; 
                float dx[2] = { Jci[0][0]*e[0] + Jci[0][1]*e[1],
                                Jci[1][0]*e[0] + Jci[1][1]*e[1] };
                x_c_FK[0] -= dx[0];
                x_c_FK[1] -= dx[1];
                break; 
            }

            float dx[2] = { Jci[0][0]*e[0] + Jci[0][1]*e[1],
                            Jci[1][0]*e[0] + Jci[1][1]*e[1] };

            x_c_FK[0] -= dx[0];
            x_c_FK[1] -= dx[1];

            //std::cout << it << '\n';
        }
    }

private:
    void build_rB_rC(float qr, float qp, float theta_ref[2])
    {
        float sx = std::sin(qr), cx = std::cos(qr);
        float sy = std::sin(qp), cy = std::cos(qp);

        float x_rot[3][3] = {
            {  cy,  sy*sx,  sy*cx },
            {   0,       cx,   -sx },
            { -sy,  cy*sx,  cy*cx }
        };

        for (int j = 0; j < 2; ++j)
        {
            float cb = std::cos(theta_ref[j]);
            float sb = std::sin(theta_ref[j]);

            float d[3] = { r_B0[0][j]-r_A0[0][j], r_B0[1][j]-r_A0[1][j], r_B0[2][j]-r_A0[2][j] };
            float p[3] = {  cb*d[0] + sb*d[2], d[1], -sb*d[0] + cb*d[2] };

            for (int i = 0; i < 3; ++i)
            {
                r_B_FK[i][j] = r_A0[i][j] + p[i];
                r_C_FK[i][j] = x_rot[i][0]*r_C0[0][j] + x_rot[i][1]*r_C0[1][j] + x_rot[i][2]*r_C0[2][j];
            }
        }
    }
};

class BalanceController {
public:
    void update() {
        if (incline_front_or_rear == 1) {
            leg_kin_.compute_inclined_foot_pos(fabs(angle_[0]), yz_initial_pos_foot_rear, o_initial_pos_rear); 
            ik_leg_.compute_leg_IK();
            x_c_IK[0] = fabs(angle_[0]); x_c_IK[1] = 0; 
            ik_spine.compute_spine_IK(x_c_IK);
        }
        else if (incline_front_or_rear == 0){
            fk_spine.compute_spine_FK(theta_act);
            inclined_angle += kp_imu_pitch * angle_[0];
            leg_kin_.compute_inclined_foot_pos(fabs(inclined_angle), yz_initial_pos_foot_front, o_initial_pos_front); 
            ik_leg_.compute_leg_IK();
            x_c_IK[0] = fabs(inclined_angle); x_c_IK[1] = 0; 
            ik_spine.compute_spine_IK(x_c_IK);
        }
    }

private:
    leg_kinematics          leg_kin_;
    IK_leg                  ik_leg_;
    IK_spine                ik_spine;
    FK_spine                fk_spine;
    float kp_imu_pitch = 0.05;
    float kv_imu_pitch = kp_imu_pitch*0.1;
};

class spine_roll_inclined_plane_node : public rclcpp::Node
{
public:
    spine_roll_inclined_plane_node()
    : Node("spine_roll_inclined_plane_node"),
    kp_waist_(6.0),    kd_waist_(0.6),
    kp_hip_(6.0),      kd_hip_(0.6),
    kp_knee_(4.0),     kd_knee_(0.4),
    kp_spine_front_(8.0), kd_spine_front_(0.8),
    kp_spine_rear_(8.0),  kd_spine_rear_(0.8),
    sleep(1)
    {
        sub_ = create_subscription<yesense_interface::msg::EulerOnly>(
            "/euler_only", 1,
            [this](const yesense_interface::msg::EulerOnly::SharedPtr m)
            {
                std::lock_guard<std::mutex> lk(angles_mtx_);
                angle_[0] = m->euler.roll  * DEG2RAD;   // imu roll is robot pitch
                angle_[1] = m->euler.pitch * DEG2RAD;   // imu pitch is robot roll
                angle_[2] = m->euler.yaw   * DEG2RAD;   // yaw stays yaw
                //RCLCPP_INFO(this->get_logger(), "IMU: pitch %f, roll %f, yaw %f", angle_[0], angle_[1], angle_[2]);
            });

        imu_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "imu_data", 
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            angle_[0]     = msg->data[0];
            angle_vel[0]  = msg->data[1];
        });



        pos_sub_ = create_subscription<std_msgs::msg::String>(
            "joint_pos_topic", 10,
            [this](std_msgs::msg::String::SharedPtr m){
                std::stringstream ss(m->data);
                int leg, joint;
                double pos;
                char sep;
                ss >> leg >> sep >> joint >> sep >> pos;
                if (leg==5 && joint>=1 && joint<=3)
                theta_act[joint-1] = pos;
            });


        // --- Publishers ---
        publisher_     = create_publisher<std_msgs::msg::Float64MultiArray> ("actuator_goal_rad", 10);
        publisher_mit_ = create_publisher<std_msgs::msg::String>            ("leg_command_topic", 10);
        
        rclcpp::sleep_for(1s);
        mit_zero();
        rclcpp::sleep_for(4s);

        timer_ctrl_ = create_wall_timer(50ms, [this]() {
            for (int j=1; j<=3; ++j) {
                std_msgs::msg::String req;
                req.data = std::to_string(5) + "," + std::to_string(j) + ",request";
                publisher_mit_->publish(req);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
            }
            controller_.update();

            std_msgs::msg::Float64MultiArray message;
            message.data.clear();

            // Order: legs[0][0] [1][0] [2][0] [0][1] ... [2][3], spine[0] [1] [2]
            for (int j = 0; j < 4; j++) {
                for (int i = 0; i < 3; i++) {
                    message.data.push_back(legs_actuator_goal_rad[i][j]);
                }
            }
            for (int i = 0; i < 3; i++) {
                message.data.push_back(spine_actuator_goal_rad[i]);
            }

            std::ostringstream oss;
            for (const auto& val : message.data) { oss << val << " "; }
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", oss.str().c_str());

            publisher_->publish(message);
        });

        timer_mit_ = create_wall_timer(50ms, [this] { send_MIT_cmd(); });
    }

private:
    // --- IMU subscriber and state cache ---
    rclcpp::Subscription<yesense_interface::msg::EulerOnly>::SharedPtr sub_;
    std::mutex angles_mtx_;

    // --- Subsrcibers ---
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr imu_sub_;

    // --- Publishers and timers ---
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>          ::SharedPtr publisher_mit_;
    rclcpp::TimerBase::SharedPtr timer_ctrl_;
    rclcpp::TimerBase::SharedPtr timer_mit_;

    // --- Controller ---
    BalanceController controller_;

    // --- Gains and sleep ---
    double kp_waist_, kd_waist_, kp_hip_, kd_hip_, kp_knee_, kd_knee_;
    double kp_spine_front_, kd_spine_front_, kp_spine_rear_, kd_spine_rear_;
    int    sleep;

    static std::string fmt(int leg, int joint, const std::string& tail)
    {
    return std::to_string(leg) + "," +
            std::to_string(joint) + "," + tail;
    }

    void mit_zero()
    {
        std_msgs::msg::String cmd;
        int l;

        for (int l = 1; l < 5; ++l)    {        
            for (int j = 1; j <= 3; ++j) {      
                cmd.data = fmt(l, j, "enable");
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(2));
                cmd.data = fmt(l, j, "mit,6,0.6,0,0,0");
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(2));
            }
        }
        l = 5;
        for (int j = 1; j <= 3; ++j) {      
                cmd.data = fmt(l, j, "enable");
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(2));
                cmd.data = fmt(l, j, "mit,10,1,0,0,0");
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(2));
            }
    }

    void send_MIT_cmd(){
        std_msgs::msg::String cmd;
        for (int leg = 1; leg < 5; ++leg){
            cmd.data = mit_prepare(leg, 1, kp_waist_, kd_waist_, legs_actuator_goal_rad[0][leg-1], 0, 0);
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
            cmd.data = mit_prepare(leg, 2, kp_hip_, kd_hip_, legs_actuator_goal_rad[1][leg-1], 0, 0);
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
            cmd.data = mit_prepare(leg, 3, kp_knee_, kd_knee_, 1.5*legs_actuator_goal_rad[2][leg-1], 0, 0);
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
        }
        int l = 5;
        cmd.data = mit_prepare(l, 1, kp_spine_front_, kd_spine_front_, spine_actuator_goal_rad[0], 0, 0);
            publisher_mit_->publish(cmd);
            rclcpp::sleep_for(std::chrono::milliseconds(sleep));
        cmd.data = mit_prepare(l, 2, kp_spine_rear_, kd_spine_rear_, spine_actuator_goal_rad[1], 0, 0);
            publisher_mit_->publish(cmd);
            rclcpp::sleep_for(std::chrono::milliseconds(sleep));
        cmd.data = mit_prepare(l, 3, kp_spine_rear_, kd_spine_rear_, spine_actuator_goal_rad[2], 0, 0);
            publisher_mit_->publish(cmd);
            rclcpp::sleep_for(std::chrono::milliseconds(sleep));
    }

    static std::string mit_prepare(int leg, int joint, double kp, double kd, double q, double dq, double tau)
    {
        return std::to_string(leg) + "," +
                std::to_string(joint) + "," +
                "mit," +
                std::to_string(kp) + "," +
                std::to_string(kd) + "," +
                std::to_string(q) + "," +
                std::to_string(dq) + "," +
                std::to_string(tau);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<spine_roll_inclined_plane_node>());
    rclcpp::shutdown();
    return 0;
}
