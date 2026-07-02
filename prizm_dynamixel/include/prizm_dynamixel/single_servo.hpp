#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rcutils/cmdline_parser.h>

#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "prizm_interfaces/msg/dynamixel_pos.hpp"
#include "prizm_interfaces/srv/dynamixel_get_pos.hpp"
using std::placeholders::_1;
using std::placeholders::_2;


typedef prizm_interfaces::msg::DynamixelPos DynaPos;
typedef prizm_interfaces::srv::DynamixelGetPos DynaGetPos;

class SingleServo : public rclcpp::Node {
   public:
    SingleServo(dynamixel::PortHandler* portHandler,
                dynamixel::PacketHandler* packetHandler);
    virtual ~SingleServo();

   private:
    rclcpp::Subscription<DynaPos>::SharedPtr position_sub_;
    rclcpp::Service<DynaGetPos>::SharedPtr get_position_srv_;
    void set_position_callback(const DynaPos::SharedPtr msg);
    void get_position_service(std::shared_ptr<DynaGetPos::Request> request,
                      std::shared_ptr<DynaGetPos::Response> response);

    int current_pos;
    dynamixel::PortHandler* port_handler_;
    dynamixel::PacketHandler* packet_handler_;
};

int main(int argc, char* argv[]);