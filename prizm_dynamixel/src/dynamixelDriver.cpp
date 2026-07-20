#include <prizm_dynamixel/dynamixelDriver.hpp>

DynamixelDriver::DynamixelDriver() : Node("DynaDriver") {}

DynamixelDriver::DynamixelDriver(std::string& nodeName,
                                 rclcpp::NodeOptions& options)
    : Node(nodeName, options) {
    this->declare_parameter(paramNamespace_ + ".device_addr", "/dev/ttyUSBX");
    address = this->get_parameter(paramNamespace_ + ".device_addr").as_string();
    // RCLCPP_INFO(this->get_logger(), "Device port: %s", address.c_str());

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

    auto ids_parameter_map = std::map<std::string, std::vector<int64_t>>{
        {"servo_ids", std::vector<int64_t>()},
        {"motor_ids", std::vector<int64_t>()},
    };
    this->declare_parameters(paramNamespace_, ids_parameter_map);
    servoIdsINT_ =
        this->get_parameter(paramNamespace_ + ".servo_ids").as_integer_array();
    motorIdsINT_ =
        this->get_parameter(paramNamespace_ + ".motor_ids").as_integer_array();

    InitMotors();
}

void DynamixelDriver::InitMotors() {
    // check that we have as many IDs as we have motors

    try {
        if (numServos_ != (int)servoIdsINT_.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Mismatch in number of servos: %d and number of servo ids: %d",
                numServos_, (int)servoIdsINT_.size());
            throw std::runtime_error("ServoNumMismatch");
        } else {
            servoList_.resize(numServos_);
            for (int i = 0; i < numServos_; i++) {
                servoList_[i].motorId_ = servoIdsINT_[i];
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    try {
        if (numMotors_ != (int)motorIdsINT_.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Mismatch in number of motors: %d and number of motor ids: %d",
                numMotors_, (int)motorIdsINT_.size());
            throw std::runtime_error("MotorNumMismatch");
        } else {
            motorList_.resize(numMotors_);
            for (int i = 0; i < numMotors_; i++) {
                motorList_[i].motorId_ = motorIdsINT_[i];
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

void DynamixelDriver::InitDynamixelComms() {
    // set up porthandler
    portHandler_ = dynamixel::PortHandler::getPortHandler(address.c_str());
    // set up packethandler
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler();

    // set up bulkreader
    groupBulkRead_ = std::make_shared<dynamixel::GroupBulkRead>(portHandler_,
                                                                packetHandler_);

    // setup bulkwriter
    groupBulkWrite_ = std::make_shared<dynamixel::GroupBulkWrite>(
        portHandler_, packetHandler_);
}

void DynamixelDriver::InitGoalSubscribers() {
    if (numServos_ != 0) {
        servoGoalPosSub_.resize(numServos_);
        std::string topic_name = "servo";
        for (int i = 0; i < numServos_; i++) {
            topic_name = "servo_" + std::to_string(i);
            servoGoalPosSub_[i] =
                this->create_subscription<prizm_interfaces::msg::DynamixelPos>(
                    topic_name, 10,
                    std::bind(&DynamixelDriver::servoPosGoalCB_, this,
                              std::placeholders::_1));
        }
    }

    if (numMotors_ != 0) {
        motorGoalPosSub_.resize(numMotors_);
        for (int i = 0; i < numMotors_; i++) {
            std::string topic_name = "motor";
            topic_name = "motor_" + std::to_string(i);
            motorGoalPosSub_[i] =
                this->create_subscription<prizm_interfaces::msg::DynamixelVel>(
                    topic_name, 10,
                    std::bind(&DynamixelDriver::motorPosGoalCB_, this,
                              std::placeholders::_1));
        }
    }
}

void DynamixelDriver::servoPosGoalCB_(const prizm_interfaces::msg::DynamixelPos& msg) {
    int id = msg.id;
    servoList_[id].goalPos_ = msg.position;
}
void DynamixelDriver::motorPosGoalCB_(const prizm_interfaces::msg::DynamixelVel& msg) {
    int id = msg.id;
    motorList_[id].goalVel_ = msg.velocity;
}

