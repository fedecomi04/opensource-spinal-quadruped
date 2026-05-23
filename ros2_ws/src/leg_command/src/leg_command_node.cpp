#include <cstdio>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>      
#include <optional> 
#include <iomanip>
#include <tuple>
#include <exception>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
constexpr float DEG2RAD = static_cast<float>(M_PI / 180.0);

class leg_commands 
{
public : 

    explicit leg_commands(const rclcpp::Logger &logger, int &fd_front, int &fd_rear)
    : logger_(logger), fd_front_(fd_front), fd_rear_(fd_rear) {}

        std::optional<std::tuple<int,int,float>> handle_cmd(const std::string &txt) {
        std::stringstream ss(txt); std::string tok; std::vector<std::string> request;
        while(std::getline(ss,tok,',')) request.push_back(tok); 
        
        std::array<uint8_t, 8>  can_payload;          // filled in prepare_payload_cmd/mit/request
        std::array<uint8_t, 11> can_frame{};          // whole FDCAN frame

        int can_bus = std::stoi(request[0]); int motor_id = std::stoi(request[1]); std::string cmd = request[2]; int can_id = motor_id;

        if (cmd == "mit" && request.size() == 8) prepare_payload_mit(can_payload, request, can_bus, can_id);
        else if (cmd == "request") prepare_payload_request(can_payload, request, can_id);
        else if (request.size() == 3) prepare_payload_cmd(cmd, can_payload);
        else  throw std::runtime_error("unknown command");

        build_frame(can_bus, can_id, can_payload, can_frame);
        bool is_request = (cmd == "request");
        auto pos = send_usb(can_bus, can_frame, is_request);
        if (is_request && pos) return std::make_tuple(can_bus, motor_id, *pos);
        return std::nullopt;
    }

private :
    rclcpp::Logger logger_;
    int &fd_front_, &fd_rear_;
    
    inline uint16_t f2u(float x, float xmin, float xmax, unsigned bits)
    {
        x = std::clamp(x, xmin, xmax);
        float span  = xmax - xmin;
        float ratio = (x - xmin) / span;
      return static_cast<uint16_t>(std::round(ratio * ((1u << bits) - 1)));
    }

    inline uint8_t to_u8(int v) { return static_cast<uint8_t>(v & 0xFF); }

    inline std::array<uint8_t,14> build_usb_frame(const std::array<uint8_t,11>& can_frame)
    {
    std::array<uint8_t,14> usb{};

    usb[0] = 0xAA;            // start-of-frame
    usb[1] = 0x09;            // fixed value do NOT change

    std::copy(can_frame.begin(), can_frame.end(), usb.begin() + 2);

    uint16_t sum = 0;        
    for (int i = 2; i <= 12; ++i) sum += usb[i];
    usb[13] = static_cast<uint8_t>(sum & 0xFF);

    return usb;
    }


    void prepare_payload_cmd(const std::string& cmd,std::array<uint8_t,8>& can_payload) {
    can_payload.fill(0xFF);

    if      (cmd == "enable")             can_payload[7] = 0xFC;
    else if (cmd == "disable")            can_payload[7] = 0xFD;
    else if (cmd == "set_zero_position")  can_payload[7] = 0xFE;
    else  throw std::runtime_error("bad cmd (not enable, nor disable, nor set_zero_position): " + cmd);
    }

    void prepare_payload_mit(std::array<uint8_t,8>& can_payload, const std::vector<std::string>& request, const int can_bus, const int can_id) {
        float kp = std::stof(request[3]), kd = std::stof(request[4]);
        float q  = std::stof(request[5]), dq = std::stof(request[6]), tau = std::stof(request[7]);
        double q_max, dq_max, tau_max;

        if (std::isnan(q)) {
            RCLCPP_WARN(logger_, "Rejected MIT command: q is NaN");
            throw std::runtime_error("position is NaN");
        }

        float q_limit_min = -M_PI;
        float q_limit_max = M_PI;
        if (can_bus == 5) {
            q_limit_min = - 40*DEG2RAD ; q_limit_max = 40*DEG2RAD;
        }
        else {
            if      (can_id == 1) { q_limit_min = -30*DEG2RAD; q_limit_max = 30*DEG2RAD;}
            else if (can_id == 2) { q_limit_min = -60*DEG2RAD; q_limit_max = 60*DEG2RAD;}
            else if (can_id == 3) { q_limit_min = -135*DEG2RAD; q_limit_max = 135*DEG2RAD;}
        }
        
        if (q < q_limit_min || q > q_limit_max) {
            RCLCPP_WARN(logger_, "Rejected MIT command for leg %d, joint %d: q=%.3f out of range [%.2f, %.2f]", can_bus, can_id, q, q_limit_min, q_limit_max);
            throw std::runtime_error("position out of range");
        }


        if (can_bus != 5){q_max = 12.5, dq_max = 65.0, tau_max = 18.0; // legs
                            if (can_id == 2 || can_id == 3) { q_max = 12.5; dq_max = 100.0; tau_max = 30.0; }}
        else {q_max = 12.5; dq_max = 100.0; tau_max = 30.0; //spine
                            if (can_id == 2 || can_id == 3) { q_max = 12.5, dq_max = 65.0, tau_max = 18.0; }}

        uint16_t q_u   = f2u(q,  -q_max, q_max, 16);
        uint16_t dq_u  = f2u(dq, -dq_max, dq_max, 12);
        uint16_t tau_u = f2u(tau, -tau_max, tau_max, 12);
        uint16_t kp_u  = f2u(kp, 0.f, 50.f, 12);
        uint16_t kd_u  = f2u(kd, 0.f,   5.f, 12);

        can_payload[0] = q_u >> 8;   
        can_payload[1] = q_u;
        can_payload[2] = dq_u >> 4;
        can_payload[3] = ((dq_u & 0xF) << 4) | (kp_u >> 8);
        can_payload[4] = kp_u;
        can_payload[5] = kd_u >> 4;
        can_payload[6] = ((kd_u & 0xF) << 4) | (tau_u >> 8);
        can_payload[7] = tau_u;
    }

    void prepare_payload_request(std::array<uint8_t,8>& can_payload, const std::vector<std::string>& request, int& can_id )
{
    can_payload.fill(0x00);                        
    uint16_t motor_id = static_cast<uint16_t>(std::stoi(request[1]));
    uint8_t  reg_id   = (request.size() > 3) ? to_u8(std::stoi(request[3])) : 80;                      

    can_payload[0] = to_u8(motor_id);               
    can_payload[1] = to_u8(motor_id >> 8);          
    can_payload[2] = 0x33;                          
    can_payload[3] = reg_id;                        

    can_id = 0x7FF;                                 
}


    void build_frame(int int_can_bus,int int_can_id, const std::array<uint8_t,8>& can_payload, std::array<uint8_t, 11>& can_frame){
    if (int_can_bus == 3) int_can_bus =1;
    if (int_can_bus == 4) int_can_bus =2;
    if (int_can_bus == 5) int_can_bus =3;

    uint8_t  can_bus = to_u8(int_can_bus);  
    uint16_t can_id  = static_cast<uint16_t>(int_can_id) & 0x07FF; // for security zero-out bits 15-11 
    can_frame[0] = can_bus;
    can_frame[1] = static_cast<uint8_t>((can_id >> 8) & 0x07);   // bits 10-8
    can_frame[2] = static_cast<uint8_t>(can_id & 0xFF);          // bits  7-0
    std::copy(can_payload.begin(), can_payload.end(), can_frame.begin()+3);
    }

    void dump_usb_frame(const std::array<uint8_t,14>& usb_frame)
{
    std::ostringstream out;
    out << "USB frame: ";
    for (uint8_t b : usb_frame)
        out << "0x" << std::hex << std::uppercase
            << std::setw(2) << std::setfill('0')
            << static_cast<int>(b) << ',';
    RCLCPP_INFO(rclcpp::get_logger("debug"), "%s", out.str().c_str());
}

    std::optional<float> send_usb(
        int leg,
        const std::array<uint8_t,11>& can_frame,
        bool wait_reply)
{
    auto usb_frame = build_usb_frame(can_frame);

    int &fd = (leg <= 2 ? fd_front_ : fd_rear_);
    if (fd < 0) {
        RCLCPP_ERROR(logger_, "Serial FD not open for leg %d", leg);
        return std::nullopt;
    }

    // Write entire USB frame
    {
    std::ostringstream oss;
    oss << std::hex << std::uppercase << std::setfill('0');
    oss << "USB frame:";
    for (uint8_t b : usb_frame) {
        oss << ' ' << std::setw(2) << int(b);
    }
    RCLCPP_INFO_STREAM(logger_, oss.str());
    }

    size_t sent = 0;
    while (sent < usb_frame.size()) {
        ssize_t n = ::write(fd, usb_frame.data() + sent, usb_frame.size() - sent);
        if (n <= 0) {
            perror("write");
            return std::nullopt;
        }
        sent += n;
    }

    if (!wait_reply)
        return std::nullopt;

    // Read up to 4 bytes (VMIN=0, VTIME set in init_ports -> timeout)
    uint8_t buf[4];
    size_t got = 0;
    while (got < 4) {
        ssize_t n = ::read(fd, buf + got, 4 - got);
        if (n > 0) {
            got += n;
        } else if (n == 0) {
            // VTIME expired
            RCLCPP_WARN(logger_, "Timeout waiting for reply (%zu/4 bytes)", got);
            return std::nullopt;
        } else {
            perror("read");
            return std::nullopt;
        }
    }

    // Unpack float
    float pos;
    std::memcpy(&pos, buf, sizeof(pos));
    RCLCPP_INFO(logger_, "Received position: %.3f rad", pos);
    return pos;
}
};

class LegCommandNode : public rclcpp::Node
{
public:
  LegCommandNode(): Node("leg_command_node"), fd_front_(-1), fd_rear_(-1),
   commander_(this->get_logger(), fd_front_, fd_rear_)
  {
    pub_ = create_publisher<std_msgs::msg::String>("joint_pos_topic", 10);

    sub_ = create_subscription<std_msgs::msg::String>(
    "leg_command_topic",
    10,
    [this](std_msgs::msg::String::UniquePtr msg)
    {
        try
        {
            auto result = commander_.handle_cmd(msg->data);
            if (!result)
                return;   // nothing to publish this time

            auto [leg, joint, pos] = *result;

            std_msgs::msg::String out;
            out.data = std::to_string(leg) + "," +
                       std::to_string(joint) + "," +
                       std::to_string(pos);
            pub_->publish(out);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "handle_cmd failed for \"%s\": %s",
                         msg->data.c_str(),
                         e.what());
        }
    });

      

      init_ports();
  }

  ~LegCommandNode() override {
        if (fd_front_ >= 0) ::close(fd_front_);
        if (fd_rear_  >= 0) ::close(fd_rear_);
    }

private:
    int fd_front_;  
    int fd_rear_;    
    leg_commands commander_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     pub_;


    void init_ports() {
    const int flags = O_RDWR | O_NOCTTY | O_SYNC;

    fd_front_ = ::open("/dev/ttyACM0", flags);
    if (fd_front_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open /dev/ttyACM0: %s", std::strerror(errno));
    } else if (!configure_port(fd_front_)) {
      RCLCPP_ERROR(get_logger(), "Failed to configure /dev/ttyACM0");
      ::close(fd_front_); fd_front_ = -1;
    }

    fd_rear_ = ::open("/dev/ttyACM1", flags);
    if (fd_rear_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open /dev/ttyACM1: %s", std::strerror(errno));
    } else if (!configure_port(fd_rear_)) {
      RCLCPP_ERROR(get_logger(), "Failed to configure /dev/ttyACM1");
      ::close(fd_rear_); fd_rear_ = -1;
    }
  }

  static bool configure_port(int fd) {
    termios tio{};
    if (tcgetattr(fd, &tio) < 0) return false;
    cfmakeraw(&tio);
    cfsetspeed(&tio, B115200);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN]  = 0;  // non-blocking read
    tio.c_cc[VTIME] = 1;  // value * 100 ms timeout
    return tcsetattr(fd, TCSANOW, &tio) == 0;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegCommandNode>());
  rclcpp::shutdown();
  return 0;
}