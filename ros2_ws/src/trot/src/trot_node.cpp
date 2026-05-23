#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <sstream>
#include <thread>
#include <algorithm> 
#define _USE_MATH_DEFINES       
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

    // VALID FOR ALL: leg*3 + joint (order: FR, FL, RR, RL) && spine (order: F, RR, RL)
    double xyz_goal_mm[3][4] = {0};
    double legs_actuator_goal_rad[3][4] = {0};
    double spine_actuator_goal_rad[3] = {0};
    const int leg = 4;
    const int joint = 3;
    const int spine = 3;

class TrotEquations {
public: 
    
    void compute_xyz_goal_pos(){
    static rclcpp::Clock steady_clock{RCL_STEADY_TIME};     // monotonic, never jumps
    static const rclcpp::Time t0 = steady_clock.now();      // captured once
    double t_current = (steady_clock.now() - t0).seconds(); 

    
        for (int i = 0; i < leg; i++){
            t_normalized[i] = fmod(t_current / T_stride + phi[i], 1.0) * T_stride;
            if (t_normalized[i] < T_normalized_stance) stance(t_normalized[i], i, T_normalized_stance);   
            else                                       swing(t_normalized[i], i, T_normalized_stance);
        }
    }


private: 
    // TIME CONSTANTS [s] (if not specified)
    const double T_stride = 2;                      // stride period
    const double B = 0.65;                             // duty factor [%/100]
    const double T_normalized_stance = T_stride * B;
    const double phi[leg] = {0, 0.5, 0.5, 0};         // fraction by which the movement is offset for a leg [%/100]

    // GEOMETRY CONSTANTS [mmm] (if not specified)
    const double Z_base_height = - 0.255;
    const double X_goal_endeffector = 0.06;
    const double HEIGHT_apex_swing = 0.02;      // swing apex height  
    const double LENGTH_stride=0.06;                         // stride length                                [m]

    // VARIABLES
    double t_normalized[leg] = {0};             // normalized over a stride period T [-]

    double smooth_poly(double u)            // u in [0,1]
    {
        return 10*u*u*u - 15*u*u*u*u + 6*u*u*u*u*u;
    }

    void stance(double t, int i, double T_normalized_stance){
        double t_stance = t / T_normalized_stance;
        x_goal(i);  
        y_goal(i, 0, t_stance);                                
        xyz_goal_mm[2][i] = Z_base_height;
}

    void swing(double t, int i, double T_normalized_stance){
        double t_swing = (t - T_normalized_stance) / (T_stride - T_normalized_stance);
        x_goal(i);  
        y_goal(i, 1, t_swing);                            
        xyz_goal_mm[2][i] = Z_base_height + ((HEIGHT_apex_swing/1.875) * (30*pow(t_swing, 2) - 60*pow(t_swing, 3) + 30*pow(t_swing,4)));
}

    void x_goal(int i){
        if (i == 0 || i == 3) xyz_goal_mm[0][i] = X_goal_endeffector;
        else xyz_goal_mm[0][i] = - X_goal_endeffector;
}

    void y_goal(int i, int s, double t_s){ 
        double y_stance = (LENGTH_stride/2 - LENGTH_stride * smooth_poly(t_s));
        double y_swing = -LENGTH_stride/2 + LENGTH_stride * smooth_poly(t_s);

        if (s == 0) {                    // stance
            if (i == 2 || i == 3)       xyz_goal_mm[1][i] = (- y_stance);
            else if (i == 0 || i == 1)  xyz_goal_mm[1][i] = (  y_stance); 
        }
        if (s == 1) {                    // swing
            if (i == 2 || i == 3)       xyz_goal_mm[1][i] = (- y_swing);
            else if (i == 0 || i == 1)  xyz_goal_mm[1][i] = (  y_swing);
        }
    }  
};

class IKSolver {
public:
    
    void compute_IK(){ 
        // in modern robotics: x_MR = x_me, y_MR= - z_me, z_MR = y_me, from MR to me frame: + 90� x rotation of the MR frame.

        for (int leg = 0; leg < 4; leg ++) {
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
    }


private: 
    const double d1 = 0.06;     // x-offset (waist to hip)
    const double a2 = 0.14;     // femur length (hip to knee length)
    const double a3 = 0.141;    // tibia length (knee to end effector length)

};

class SpineEquations{
public: 
    void compute_spine_actuator_goal(){
        for (int i = 0; i < spine; i++) spine_actuator_goal_rad[i] = 0;
    }
};

class GaitController {
public:
    void update() {
        trot_eq_.compute_xyz_goal_pos();
        ik_.compute_IK();
        spine_eq_.compute_spine_actuator_goal();
    }
private:
    TrotEquations   trot_eq_;
    IKSolver        ik_;
    SpineEquations  spine_eq_;
};

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher(GaitController* controller)          
    : Node("trot_node"),
    controller_(controller),
    kp_waist_(0.0),  kd_waist_(0.0),
    kp_hip_(0.0),    kd_hip_(0.0),
    kp_knee_(0.0),   kd_knee_(0.0),
    kp_spine_front_(0.0), kd_spine_front_(0.0),
    kp_spine_rear_(0.0),  kd_spine_rear_(0.0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("actuator_goal_rad", 10);
        timer_ = this->create_wall_timer(10ms, [this]() {
            controller_->update();

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

        publisher_mit_ = create_publisher<std_msgs::msg::String>(
                        "leg_command_topic", 10);
        
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        
        //setup_sequence(); 
        mit_zero();
        rclcpp::sleep_for(4s);
        ramp_start_ = now();
        timer_mit_ = create_wall_timer(30ms, [this] { send_MIT_cmd(); });
    }

    void shutdown_motors() { send_disable_all(); }
    
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_mit_;
    rclcpp::Time ramp_start_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;  // for MuJoCO
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_mit_;         // for real quadruped (leg_command_topic)

    GaitController* controller_;

    double kp_waist_, kd_waist_;
    double kp_hip_,   kd_hip_;
    double kp_knee_,  kd_knee_;
    double kp_spine_front_, kd_spine_front_;
    double kp_spine_rear_,  kd_spine_rear_;

    const int sleep = 1;

    static std::string fmt(int leg, int joint, const std::string& tail)
    {
    return std::to_string(leg) + "," +
            std::to_string(joint) + "," + tail;
    }

    void setup_sequence() 
    {
        do_startup_sequence(6); 
        rclcpp::sleep_for(5s);
        do_startup_sequence(1); 
        rclcpp::sleep_for(5s);
        do_startup_sequence(2); 
        rclcpp::sleep_for(5s);
        do_startup_sequence(3); 
        rclcpp::sleep_for(5s);
        do_startup_sequence(4); 
        rclcpp::sleep_for(5s);
        do_startup_sequence(5); 
        rclcpp::sleep_for(5s);
    }

    void mit_zero()
    {
        std_msgs::msg::String cmd;
        int l = 0;

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

    void do_startup_sequence(int setup_leg )
    {
    std_msgs::msg::String cmd;

    if (setup_leg > 5) {
        for (int leg = 1; leg < 6; ++leg) {
            for (int joint = 1; joint < 4; ++joint) {
                cmd.data = fmt(leg, joint, "disable");
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
                printf("leg %d joint %d disabled \n", leg, joint);
                cmd.data = fmt(leg, joint, "enable");
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
                printf("leg %d joint %d enabled \n", leg, joint);
                cmd.data = fmt(leg, joint, "set_zero_position");
                publisher_mit_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
                printf("leg %d joint %d set_zero_position \n", leg, joint);
                cmd.data = fmt(leg, joint,"mit,0,0,0,0,0");
                publisher_mit_->publish(cmd);
                printf("leg %d joint %d mit \n", leg, joint);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));             
            }
        }
    }
    else {
        for (int joint = 1; joint < 4; ++joint){
            cmd.data = fmt(setup_leg, joint, "set_zero_position");
            publisher_mit_->publish(cmd);
            printf("leg %d joint %d set_zero_position \n", setup_leg, joint);
            rclcpp::sleep_for(std::chrono::milliseconds(sleep));
            cmd.data = fmt(setup_leg, joint,"mit,5.0,0.5,0,0,0");
            publisher_mit_->publish(cmd);
            printf("leg %d joint %d mit \n", setup_leg, joint);
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
    }
    }

    void send_MIT_cmd(){
        double elapsed = (now() - ramp_start_).seconds();
        double s = std::clamp(elapsed / 10.0, 0.1, 1.0);   // 0 ? 1 over 10 s

        // target gains
        kp_waist_        = 6.0 * s;   kd_waist_        = 0.6 * s;
        kp_hip_          = 6.0 * s;   kd_hip_          = 0.6 * s;
        kp_knee_         = 6.0 * s;   kd_knee_         = 0.6 * s;
        kp_spine_front_  = 10.0 * s;   kd_spine_front_  = 1.0 * s;
        kp_spine_rear_   = 10.0 * s;   kd_spine_rear_   = 1.0 * s;


        std_msgs::msg::String cmd;
        for (int leg = 1; leg < 5; ++leg){
        //int leg = 1;
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

        cmd.data = mit_prepare(2, 2, 10, 1, legs_actuator_goal_rad[1][2-1], 0, 0);
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

    void send_disable_all()
    {
    std_msgs::msg::String cmd;
    for (int leg = 1; leg < 5; ++leg)
        for (int joint = 1; joint < 4; ++joint) {
        cmd.data = std::to_string(leg) + "," + std::to_string(joint) + ",disable";
        publisher_mit_->publish(cmd);
        }
    }
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    GaitController gait; 
    auto node = std::make_shared<MinimalPublisher>(&gait);
    rclcpp::spin(node);        
    //node->shutdown_motors();   
    rclcpp::sleep_for(100ms);  
    rclcpp::shutdown();
    return 0;
}
