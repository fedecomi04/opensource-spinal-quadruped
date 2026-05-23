#include <stdbool.h> //for bool
#include <unistd.h> //for usleep
#include <math.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <chrono>  
#include "stdlib.h"
#include "string.h"
#include "sensor_ids.hpp"
#include <memory>
#include <cmath>  
#include <chrono>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
//char filename[] = "quadruped.xml";
char filename[] = "quadruped_UJ_activated.xml";

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0;
mjtNum ctrl;

// Inclined plane parameters
float T_incline = 5; // time to achieve max degree
float max_degree = 0; // wwill start at 0 and get to max_degree in T_incline seconds

// Define main arrays
double cur_actuator_pos[3][5] = {0};
double cur_actuator_vel[3][5] = {0};
double actuator_goal_rad[3][5] = {0};
double torque[3][5] = {0};

const double kp_hip = 30;
const double kd_hip = 3;
const double kp_femur = 10;
const double kd_femur = 1.0;
const double kp_knee = 10;
const double kd_knee =1;
const double kp_spine = 15;
const double kd_spine =1.5; 

static double integ_error[3][5] = {{0}};
const double ki_hip   = 0.5;
const double ki_femur = 0.3;
const double ki_knee  = 3;
const double ki_spine = 0.4;


// MuJoCo data structures
std::string package_share = ament_index_cpp::get_package_share_directory("mujoco_interface");
std::string model_path = package_share + "/model/" + filename;
char error[1000] = "";
mjModel* m = mj_loadXML(model_path.c_str(), NULL, error, 1000); 
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// holders of one step history of time and position to calculate derivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

void get_cur_actuator_pos(mjData* d, const mjModel* m){
    // gets actuator positions from the actuator sensors and fills cur_actuator_pos[3][4]
    // //<!-- FR leg -->
    cur_actuator_pos[0][0] = d->sensordata[m->sensor_adr[id_FR_hip_s_pos]]; 
    cur_actuator_pos[1][0] = d->sensordata[m->sensor_adr[id_FR_femur_s_pos]];
    cur_actuator_pos[2][0] = d->sensordata[m->sensor_adr[id_FR_tibia_s_pos]];
    // //<!-- FL leg -->
    cur_actuator_pos[0][1] = d->sensordata[m->sensor_adr[id_FL_hip_s_pos]]; 
    cur_actuator_pos[1][1] = d->sensordata[m->sensor_adr[id_FL_femur_s_pos]];
    cur_actuator_pos[2][1] = d->sensordata[m->sensor_adr[id_FL_tibia_s_pos]];   
    // //<!-- RR leg -->
    cur_actuator_pos[0][2] = d->sensordata[m->sensor_adr[id_RR_hip_s_pos]]; 
    cur_actuator_pos[1][2] = d->sensordata[m->sensor_adr[id_RR_femur_s_pos]];
    cur_actuator_pos[2][2] = d->sensordata[m->sensor_adr[id_RR_tibia_s_pos]];   
    // //<!-- RL leg -->
    cur_actuator_pos[0][3] = d->sensordata[m->sensor_adr[id_RL_hip_s_pos]]; 
    cur_actuator_pos[1][3] = d->sensordata[m->sensor_adr[id_RL_femur_s_pos]];
    cur_actuator_pos[2][3] = d->sensordata[m->sensor_adr[id_RL_tibia_s_pos]];   
    //<!-- Spine -->
    cur_actuator_pos[0][4] = d->sensordata[m->sensor_adr[id_F_spine_s_pos]]; 


    if (std::strcmp(filename, "model.xml") == 0) {
        cur_actuator_pos[1][4] = d->sensordata[m->sensor_adr[id_RR_spine_s_pos]];
        cur_actuator_pos[2][4] = d->sensordata[m->sensor_adr[id_RL_spine_s_pos]];
    }
    else { 
        cur_actuator_pos[1][4] = d->sensordata[m->sensor_adr[id_UJ_spine_s_pos]];
        cur_actuator_pos[2][4] = -d->sensordata[m->sensor_adr[id_UJ_spine_s_pos]];
    }

    
    for (int leg = 0; leg < 5; leg++) printf("leg %d: hip %.3f femur %.3f knee %.3f \n", leg, cur_actuator_pos[0][leg], cur_actuator_pos[1][leg], cur_actuator_pos[2][leg]);
}
void get_cur_actuator_vel(mjData* d, const mjModel* m){
    // gets actuator velocities from the actuator sensors and fills cur_actuator_pos[3][4]
    //<!-- FR leg -->
    cur_actuator_vel[0][0] = d->sensordata[m->sensor_adr[id_FR_hip_s_vel]]; 
    cur_actuator_vel[1][0] = d->sensordata[m->sensor_adr[id_FR_femur_s_vel]];
    cur_actuator_vel[2][0] = d->sensordata[m->sensor_adr[id_FR_tibia_s_vel]];
    //<!-- FL leg -->
    cur_actuator_vel[0][1] = d->sensordata[m->sensor_adr[id_FL_hip_s_vel]]; 
    cur_actuator_vel[1][1] = d->sensordata[m->sensor_adr[id_FL_femur_s_vel]];
    cur_actuator_vel[2][1] = d->sensordata[m->sensor_adr[id_FL_tibia_s_vel]];   
    //<!-- RR leg -->
    cur_actuator_vel[0][2] = d->sensordata[m->sensor_adr[id_RR_hip_s_vel]]; 
    cur_actuator_vel[1][2] = d->sensordata[m->sensor_adr[id_RR_femur_s_vel]];
    cur_actuator_vel[2][2] = d->sensordata[m->sensor_adr[id_RR_tibia_s_vel]];   
    //<!-- RL leg -->
    cur_actuator_vel[0][3] = d->sensordata[m->sensor_adr[id_RL_hip_s_vel]]; 
    cur_actuator_vel[1][3] = d->sensordata[m->sensor_adr[id_RL_femur_s_vel]];
    cur_actuator_vel[2][3] = d->sensordata[m->sensor_adr[id_RL_tibia_s_vel]];   
    //<!-- Spine -->
    cur_actuator_vel[0][4] = d->sensordata[m->sensor_adr[id_F_spine_s_vel]]; 

    if (std::strcmp(filename, "model.xml") == 0) {
            cur_actuator_vel[1][4] = d->sensordata[m->sensor_adr[id_RR_spine_s_vel]];
            cur_actuator_vel[2][4] = d->sensordata[m->sensor_adr[id_RL_spine_s_vel]];
    }
    else {
        cur_actuator_vel[1][4] = d->sensordata[m->sensor_adr[id_UJ_spine_s_vel]];
        cur_actuator_vel[2][4] = -d->sensordata[m->sensor_adr[id_UJ_spine_s_vel]];
    }
}
void computePIDtorque_fixed(mjData* d, const mjModel* m){
    get_cur_actuator_pos(d, m);
    get_cur_actuator_vel(d, m);

    double dt = m->opt.timestep;
    double err;

    for (int i = 0; i < 4; i++) {
            err = actuator_goal_rad[0][i] - cur_actuator_pos[0][i];
            integ_error[0][i] += err * dt;
            torque[0][i] = kp_hip   * err + ki_hip   * integ_error[0][i] - kd_hip   * cur_actuator_vel[0][i];
            err = actuator_goal_rad[1][i] - cur_actuator_pos[1][i];
            integ_error[1][i] += err * dt;
            torque[1][i] = kp_femur * err + ki_femur * integ_error[1][i] - kd_femur * cur_actuator_vel[1][i];
            err = actuator_goal_rad[2][i] - cur_actuator_pos[2][i];
            integ_error[2][i] += err * dt;
            torque[2][i] = kp_knee  * err + ki_knee  * integ_error[2][i] - kd_knee  * cur_actuator_vel[2][i];
    }

    if (std::strcmp(filename, "model.xml") == 0) {
        for (int joint = 0; joint < 3; joint++) {
            err = actuator_goal_rad[joint][4] - cur_actuator_pos[joint][4];
            integ_error[joint][4] += err * dt;
            torque[joint][4] = kp_spine * err + ki_spine * integ_error[joint][4] - kd_spine * cur_actuator_vel[joint][4];
        }
    }
    else {
        for (int joint = 0; joint < 2; joint++) {
            err = actuator_goal_rad[joint][4] - cur_actuator_pos[joint][4];
            integ_error[joint][4] += err * dt;
            torque[joint][4] = kp_spine * err + ki_spine * integ_error[joint][4] - kd_spine * cur_actuator_vel[joint][4];
        }
    }

        //int l = 1;
        //int j = 1;
    //printf("tau %.2f kp %.2f qdes %.2f cur_pos %.3f kd %.2f cur_vel %.3f \n", torque[j][l], kp_femur, actuator_goal_rad[j][l], cur_actuator_pos[j][l],kd_femur,cur_actuator_vel[j][l]);
       printf("tau %.2f kp %.2f qdes %.2f cur_pos %.3f kd %.2f cur_vel %.3f \n", torque[2][0], kp_knee, actuator_goal_rad[2][0], cur_actuator_pos[2][0],kd_knee,cur_actuator_vel[2][0]);
       
}
void computePDtorque_fixed(mjData* d, const mjModel* m) {
    get_cur_actuator_pos(d, m);
    get_cur_actuator_vel(d, m);
    for (int i = 0; i < 4; i++) {
        torque[0][i] = kp_hip   * (actuator_goal_rad[0][i] - cur_actuator_pos[0][i]) - kd_hip   * cur_actuator_vel[0][i];
        torque[1][i] = kp_femur * (actuator_goal_rad[1][i] - cur_actuator_pos[1][i]) - kd_femur * cur_actuator_vel[1][i];
        torque[2][i] = kp_knee  * (actuator_goal_rad[2][i] - cur_actuator_pos[2][i]) - kd_knee  * cur_actuator_vel[2][i];
    }
    if (std::strcmp(filename, "model.xml") == 0) {
        for (int joint = 0; joint < 3; joint++) {
            torque[joint][4] = kp_spine * (actuator_goal_rad[joint][4] - cur_actuator_pos[joint][4]) - kd_spine * cur_actuator_vel[joint][4];
        }
    } else {
        for (int joint = 0; joint < 2; joint++) {
            torque[joint][4] = kp_spine * (actuator_goal_rad[joint][4] - cur_actuator_pos[joint][4]) - kd_spine * cur_actuator_vel[joint][4];
        }
    }
    printf("tau %.2f kp %.2f qdes %.2f cur_pos %.3f kd %.2f cur_vel %.3f\n",
           torque[2][0],
           kp_knee,
           actuator_goal_rad[2][0],
           cur_actuator_pos[2][0],
           kd_knee,
           cur_actuator_vel[2][0]);
}
void send_cmd_torque(){
  //d->ctrl[1] = torque[1][0];
  //d->ctrl[2] = torque[2][0];
  
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {  
            //printf("i %zu, j %zu, torque %.4f \n", i, j, torque[j][i]);
            // if (1 < torque[j][i]  ) torque[j][i] = 0.5;    
            // if (torque[j][i] < -1 ) torque[j][i] = -0.5; 
            d->ctrl[i*3 + j] = torque[j][i];
            //printf("i %d, j %d, torque %.2f \n", i, j, torque[j][i]);
            //printf("i %d, j %d, cur pos %.2f cur vel %.2f \n", i, j, cur_actuator_pos[j][i],  cur_actuator_vel[j][i]);
        }
    }

    if (std::strcmp(filename, "model.xml") == 0) {
        for (int j = 0; j < 3; j++) {  
            //printf("i %zu, j %zu, torque %.4f \n", i, j, torque[j][i]);
            // if (1 < torque[j][i]  ) torque[j][i] = 0.5;    
            // if (torque[j][i] < -1 ) torque[j][i] = -0.5; 
            d->ctrl[12 + j] = torque[j][4];
            //printf("i %d, j %d, torque %.2f \n", i, j, torque[j][i]);
            //printf("i %d, j %d, cur pos %.2f cur vel %.2f \n", i, j, cur_actuator_pos[j][i],  cur_actuator_vel[j][i]);
        }
    }
    else{
        for (int j = 0; j < 2; j++) {  
            //printf("i %zu, j %zu, torque %.4f \n", i, j, torque[j][i]);
            // if (1 < torque[j][i]  ) torque[j][i] = 0.5;    
            // if (torque[j][i] < -1 ) torque[j][i] = -0.5; 
            d->ctrl[12 + j] = torque[j][4];
            //printf("i %d, j %d, torque %.2f \n", i, j, torque[j][i]);
            //printf("i %d, j %d, cur pos %.2f cur vel %.2f \n", i, j, cur_actuator_pos[j][i],  cur_actuator_vel[j][i]);
        }
    }

    
}
void send_cmd_pos() {
    for(int i=0;i<4;i++){for(int j=0;j<3;j++){d->ctrl[i*3+j]=actuator_goal_rad[j][i];}}
    if(std::strcmp(filename,"model.xml")==0){for(int j=0;j<3;j++){d->ctrl[12+j]=actuator_goal_rad[j][4];}} else {for(int j=0;j<2;j++){d->ctrl[12+j]=actuator_goal_rad[j][4];}}
    printf("actuator goal spine F %.3f, UJ %.3f", actuator_goal_rad[0][4],actuator_goal_rad[1][4]);
}
void incline_plane(const mjModel* m, mjData* d){
    double cur_time = d->time;\
    double max_angle = max_degree * M_PI /180;
    double target = (cur_time < T_incline) ? (cur_time/T_incline) * max_angle : max_angle;
    int id_plane_tilt = mj_name2id(m, mjOBJ_ACTUATOR, "plane_tilt");
    d->ctrl[id_plane_tilt] = target;
}
void controller(const mjModel* m, mjData* d){
        computePDtorque_fixed(d,m);
        //send_cmd_torque(); 
        send_cmd_pos();
        incline_plane(m, d);
}

class mujoco : public rclcpp::Node
{
public:
  mujoco() : Node("mujoco_node") {}

  void start_subscription()
  {
    if (actuator_sub_) {                
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Subscribing to /actuator_goal_rad …");

    imu_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("imu_data", 10);
    leg_pos_pub_ = this->create_publisher<std_msgs::msg::String>("joint_pos_topic", 10);


    actuator_sub_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "actuator_goal_rad", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
          if (msg->data.size() != 15) return;

          for (int l = 0; l < 5; l++){
            for (int i = 0; i < 3; i++){
              actuator_goal_rad[i][l] = msg->data[l * 3 + i];
              }
          }
        });

        publisher_timer_ = this->create_wall_timer(10ms, [this]() {
                const mjtNum* R = &d->site_xmat[9 * id_imu_site];
                mjtNum quat[4];
                mju_mat2Quat(quat, R);
                double pitchX     = std::atan2(2.0*(quat[0]*quat[1] + quat[2]*quat[3]),
                                            1.0 - 2.0*(quat[1]*quat[1] + quat[2]*quat[2]));
                double pitch_rate = d->sensordata[ m->sensor_adr[id_imu_gyro] + 0 ];
                std_msgs::msg::Float64MultiArray msg;
                msg.data.clear();
                msg.data.push_back(pitchX);
                msg.data.push_back(pitch_rate);
                imu_pub_->publish(msg);


                std_msgs::msg::String out;
                std::stringstream ss;
                for (int j = 0; j < 3; ++j) {
                ss.str(""); ss.clear();
                ss << 5            // leg ID 5 = spine
                    << ':' << (j+1) // joint index 1,2,3
                    << ':' << cur_actuator_pos[j][4]; 
                out.data = ss.str();
                leg_pos_pub_->publish(out);
                }
            });
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr actuator_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr leg_pos_pub_;

  rclcpp::TimerBase::SharedPtr publisher_timer_;

};


// ------------------------------------------ interactivity start ------------------------------------------
// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
    {
        // backspace: reset simulation
        if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
        {
            mj_resetData(m, d);
            mj_forward(m, d);
        }
    }
// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
    {
        // update button state
        button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
        button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
        button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

        // update mouse position
        glfwGetCursorPos(window, &lastx, &lasty);
    }
// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
    {
        // no buttons down: nothing to do
        if( !button_left && !button_middle && !button_right )
            return;

        // compute mouse displacement, save
        double dx = xpos - lastx;
        double dy = ypos - lasty;
        lastx = xpos;
        lasty = ypos;

        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        // get shift key state
        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

        // determine action based on mouse button
        mjtMouse action;
        if( button_right )
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if( button_left )
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else
            action = mjMOUSE_ZOOM;

        // move camera
        mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
    }
// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
    {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
    }
// ------------------------------------------ interactivity end ------------------------------------------

int main(int argc, const char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mujoco>();
    std::thread ros_thread([&]() {rclcpp::spin(node);});
    
    std::cout << "[DEBUG] Activating MuJoCo license..." << std::endl;
    mj_activate("mjkey.txt");

    std::cout << "[DEBUG] Loading MuJoCo XML model..." << std::endl; 
    std::string model_path = ament_index_cpp::get_package_share_directory("mujoco_interface") + "/model/" + filename;    
    std::cout << "[DEBUG] Full model path: " << model_path << std::endl;
    m = mj_loadXML(model_path.c_str(), NULL, error, 1000);
    
    if (!m) {
        std::cerr << "[ERROR] Failed to load model.xml: " << error << std::endl;
        return 1;
    }
    std::cout << "[DEBUG] Model loaded successfully." << std::endl;

    
    std::cout << "[DEBUG] Creating mjData..." << std::endl;
    d = mj_makeData(m);
    if (!d) {
        std::cerr << "[ERROR] Failed to allocate mjData." << std::endl;
        return 1;
    }
    std::cout << "[DEBUG] mjData created successfully." << std::endl;

    std::cout << "[DEBUG] Calling init_sensor_ids..." << std::endl;
    init_sensor_ids(m);
    std::cout << "[DEBUG] Sensor IDs initialized." << std::endl;
    
    std::cout << "[DEBUG] Initializing GLFW..." << std::endl;
    if (!glfwInit()) {
        std::cerr << "[ERROR] GLFW initialization failed!" << std::endl;
        return 1;
    }
    std::cout << "[DEBUG] GLFW initialized." << std::endl;

    std::cout << "[DEBUG] Creating GLFW window..." << std::endl;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo ROS2 Node", NULL, NULL);
    if (!window) {
        std::cerr << "[ERROR] Failed to create GLFW window!" << std::endl;
        return 1;
    }
    std::cout << "[DEBUG] GLFW window created." << std::endl;
    node->start_subscription();   
    
    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
  

    // create window, make OpenGL context current, request v-sync
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    mjcb_control = controller;

    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        
        cam.type        = mjCAMERA_TRACKING;                                     // switch to tracking mode :contentReference[oaicite:0]{index=0}
        cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "frontbase");                // follow this body
        cam.distance    = 1.5;                                                   // metres behind
        cam.azimuth     = 180;                                                   // degrees around
        cam.elevation   = 0; 

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW 
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    rclcpp::shutdown();
    ros_thread.join();

    return 1;
}
