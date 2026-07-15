#include <rclcpp/rclcpp.hpp>

#include "prizm_dynamixel/dynamixelMotor.hpp"

class DynamixelDriver : public DynamixelMotor, public rclcpp::Node {
   public:
    // void InitServos();
    void InitMotors();
    void InitDynamixelComms();
    // uint8_t getMotorId() override;
    // uint32_t getOpMode() override;
    // uint32_t getTemp() override;
    // uint32_t getErrorStatus() override;
    // uint32_t getCurrentPos() override;
    // uint32_t getCurrentVel() override;
    // uint32_t getCurrentAccel() override;


    std::string address = "/dev/ttyUSBX";
    std::string paramNamespace_ = "motor_driver";
    int numServos_, servoMin_, servoMax_;
    int numMotors_, speedMin_, speedMax_;
    std::vector<dynamixelData> servoList_;
    std::vector<dynamixelData> motorList_;
    std::vector<uint8_t> servoIds_;
    std::vector<uint8_t> motorIds_;
    DynamixelDriver();
    DynamixelDriver(std::string& nodeName, rclcpp::NodeOptions& options);
    
   private:
    std::string nodeName_;
    

};
