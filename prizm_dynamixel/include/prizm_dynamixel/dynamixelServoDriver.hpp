#include <rclcpp/rclcpp.hpp>

#include "prizm_dynamixel/dynamixelMotor.hpp"

typedef std::pair<rclcpp::ParameterType, rcl_interfaces::msg::ParameterDescriptor> paramPair;

class DynamixelServoDriver : public DynamixelMotor, public rclcpp::Node {
   public:
    void InitServos();
    uint8_t getMotorId();
    uint32_t getOpMode();
    uint32_t getTemp();
    uint32_t getErrorStatus();
    uint32_t getCurrentPos();
    uint32_t getCurrentVel();
    uint32_t getCurrentAccel();

   private:
    DynamixelServoDriver();
    std::string nodeName_;
    std::string address = "/dev/ttyUSB0";
    int numServos_;
    std::vector<dynamixelData> servoList_;

    rclcpp::Parameter numServosParam_, addresParam_;
};