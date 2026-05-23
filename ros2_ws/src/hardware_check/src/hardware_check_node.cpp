#include <string>
#include <chrono>          
#include <memory>          
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;   

class Publisher : public rclcpp::Node
{
public:
    Publisher() : Node("hardware_check_node")
    {
    publisher_ = create_publisher<std_msgs::msg::String>(
                    "leg_command_topic", 10);

    setup_sequence();
    
    //timer_mit_ = create_wall_timer(50ms, [this] { send_mit_cst_test(); });
    //rclcpp::sleep_for(10s); 
    //  timer_ = create_wall_timer( 80ms, [this] { send_position_requests(); }); 
    //send_disable_all();
    //rclcpp::shutdown();                     

    }
    void shutdown_motors() { send_disable_all(); }


private:
    rclcpp::TimerBase::SharedPtr timer_mit_;
    const int leg_number = 6;
    const int sleep = 1;

    void setup_sequence(){
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

    void do_startup_sequence(int setup_leg )
    {
    std_msgs::msg::String cmd;

    if (setup_leg > 5) {
        for (int leg = 1; leg < 6; ++leg) {
            for (int joint = 1; joint < 4; ++joint) {
                cmd.data = fmt(leg, joint, "disable");
                publisher_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
                printf("leg %d joint %d disabled \n", leg, joint);
                cmd.data = fmt(leg, joint, "enable");
                publisher_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
                printf("leg %d joint %d enabled \n", leg, joint);
                cmd.data = fmt(leg, joint, "set_zero_position");
                publisher_->publish(cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));
                printf("leg %d joint %d set_zero_position \n", leg, joint);
                cmd.data = fmt(leg, joint,"mit,0,0,0,0,0");
                publisher_->publish(cmd);
                printf("leg %d joint %d mit \n", leg, joint);
                rclcpp::sleep_for(std::chrono::milliseconds(sleep));             
            }
        }
    }
    else {
        for (int joint = 1; joint < 4; ++joint){
            cmd.data = fmt(setup_leg, joint, "set_zero_position");
            publisher_->publish(cmd);
            printf("leg %d joint %d set_zero_position \n", setup_leg, joint);
            rclcpp::sleep_for(std::chrono::milliseconds(sleep));
            cmd.data = fmt(setup_leg, joint,"mit,1.0,0.1,0,0,0");
            publisher_->publish(cmd);
            printf("leg %d joint %d mit \n", setup_leg, joint);
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
    }
    }

    void send_position_requests()
    {
    std_msgs::msg::String cmd;
    for (int leg = 1; leg < leg_number; ++leg)
        for (int joint =1; joint < 4; ++joint) {
        cmd.data = fmt(leg, joint, "mit,1,0.1,0.1,0,0");
        publisher_->publish(cmd);
        usleep(50000);
        }
    }

    void send_mit_cst_test()
    {
    std_msgs::msg::String cmd;
    
    for ( int leg = 1; leg < leg_number; ++leg){
        //int leg = 5;
        cmd.data = fmt(leg,1,"mit,5,0.5,0,0,0");
        publisher_->publish(cmd);
        //printf("leg %d joint %d mit \n", leg, 1);
        cmd.data = fmt(leg,2,"mit,5,0.5,0,0,0");
        publisher_->publish(cmd);
        //printf("leg %d joint %d mit \n", leg, 2);
        cmd.data = fmt(leg,3,"mit,5,0.5,0,0,0");
        publisher_->publish(cmd);
        //printf("leg %d joint %d mit \n", leg, 3);
        }
    }

    void send_disable_all()
    {
    std_msgs::msg::String cmd;
    for (int leg = 1; leg < leg_number; ++leg)
        for (int joint = 1; joint < 4; ++joint) {
        cmd.data = fmt(leg, joint, "disable");
        publisher_->publish(cmd);
        }
    } 

    static std::string fmt(int leg, int joint, const std::string& tail)
    {
    return std::to_string(leg) + "," +
            std::to_string(joint) + "," + tail;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);              
    auto node = std::make_shared<Publisher>();
    rclcpp::spin(node);                     
    rclcpp::shutdown();                     
    return 0;
}
