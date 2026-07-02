#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rcutils/cmdline_parser.h>

#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "prizm_interfaces/msg/dynamixel_vel.hpp"
#include "prizm_interfaces/srv/dynamixel_get_vel.hpp"
using std::placeholders::_1;
using std::placeholders::_2;


typedef prizm_interfaces::msg::DynamixelVel DynaVel;
typedef prizm_interfaces::srv::DynamixelGetVel DynaGetVel;

class SingleMotor : public rclcpp::Node {
   public:
    SingleMotor(dynamixel::PortHandler* portHandler,
                dynamixel::PacketHandler* packetHandler);
    virtual ~SingleMotor();

   private:
    rclcpp::Subscription<DynaVel>::SharedPtr position_sub_;
    rclcpp::Service<DynaGetVel>::SharedPtr get_position_srv_;
    void set_position_callback(const DynaVel::SharedPtr msg);
    void get_position_service(std::shared_ptr<DynaGetVel::Request> request,
                      std::shared_ptr<DynaGetVel::Response> response);

    int current_pos;
    dynamixel::PortHandler* port_handler_;
    dynamixel::PacketHandler* packet_handler_;
};

int main(int argc, char* argv[]);