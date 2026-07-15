#include <prizm_dynamixel/dynamixelDriver.hpp>

DynamixelDriver::DynamixelDriver() : Node("DynaDriver") {}

DynamixelDriver::DynamixelDriver(std::string& nodeName,
                                 rclcpp::NodeOptions& options)
    : Node(nodeName, options) {
    this->declare_parameter(paramNamespace_ + ".device_addr", "/dev/ttyUSBX");
    address = this->get_parameter(paramNamespace_ + ".device_addr").as_string();

    auto servo_parameter_map = std::map<std::string, int>{
        {"num_servos", 1},
        {"servo_min", 0},
        {"servo_max", 360},
    };
    this->declare_parameters(paramNamespace_, servo_parameter_map);
    numServos_ = this->get_parameter(paramNamespace_ + ".num_servos").as_int();
    servoMin_ = this->get_parameter(paramNamespace_ + ".servo_min").as_int();
    servoMax_ = this->get_parameter(paramNamespace_ + ".servo_max").as_int();

    auto motor_parameter_map = std::map<std::string, int>{
        {"num_motors", 1},
        {"speed_min", 0},
        {"speed_max", 5},
    };

    this->declare_parameters(paramNamespace_, motor_parameter_map);
    numMotors_ = this->get_parameter(paramNamespace_ + ".num_motors").as_int();
    speedMin_ = this->get_parameter(paramNamespace_ + ".speed_min").as_int();
    speedMax_ = this->get_parameter(paramNamespace_ + ".speed_max").as_int();

    auto ids_parameter_map = std::map<std::string, std::vector<uint8_t>>{
        {"servo_ids", std::vector<uint8_t>()},
        {"motor_ids", std::vector<uint8_t>()},
    };
    this->declare_parameters(paramNamespace_, ids_parameter_map);
    servoIds_ =
        this->get_parameter(paramNamespace_ + ".servo_ids").as_byte_array();
    motorIds_ =
        this->get_parameter(paramNamespace_ + ".motor_ids").as_byte_array();

    InitMotors();
}

void DynamixelDriver::InitMotors() {
    // check that we have as many IDs as we have motors

    try {
        if (numServos_ != (int)servoIds_.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Mismatch in number of servos: %d and number of servo ids: %d",
                numServos_, (int)servoIds_.size());
            throw std::runtime_error("ServoNumMismatch");
        }
        else{
            servoList_.resize(numServos_);
            for(int i = 0; i < numServos_; i++){
                servoList_[i].motorId_ = servoIds_[i];
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    try {
        if (numMotors_ != (int)motorIds_.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Mismatch in number of servos: %d and number of servo ids: %d",
                numMotors_, (int) motorIds_.size());
            throw std::runtime_error("MotorNumMismatch");
        }
        else{
            motorList_.resize(numMotors_);
            for(int i = 0; i < numMotors_; i++){
                motorList_[i].motorId_ = motorIds_[i];
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

void DynamixelDriver::InitDynamixelComms() {
    // set up packethandler
    // set up porthandler
    // set up bulkreader
    // setup bulkwriter
}
// #ifndef BUILD_TESTING

// int main(int argc, char* argv[]){
//     rclcpp::init(argc, argv);
//     RCLCPP_INFO(rclcpp::get_logger("servo_driver"), "Started node");

//     auto servo_driver = std::make_shared<DynamixelDriver>();

//     rclcpp::spin(servo_driver);

//     rclcpp::shutdown();

//     return 0;
// }

// #endif