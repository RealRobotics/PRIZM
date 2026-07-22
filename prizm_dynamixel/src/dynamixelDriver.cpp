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

    // InitDynamixelComms();
    InitMotors();
    InitGoalSubscribers();
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

    // Enable Torque for all motors
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
        servoGoalPosSub_ = this->create_subscription<DynaPosArr>(
            "servo_pos", 10,
            std::bind(&DynamixelDriver::servoPosGoalCB_, this,
                      std::placeholders::_1));
    }

    if (numMotors_ != 0) {
        motorGoalPosSub_ = this->create_subscription<DynaVelArr>(
            "motor_vel", 10,
            std::bind(&DynamixelDriver::motorVelGoalCB_, this,
                      std::placeholders::_1));
    }
}

void DynamixelDriver::servoPosGoalCB_(const DynaPosArr& msg) {
    // 1. check if the number of servo positions received is the same as the
    // number of servos loaded from config

    int num_incoming_pos = msg.pos_array.size();
    try {
        if (num_incoming_pos != numServos_) {
            RCLCPP_ERROR(this->get_logger(),
                         "Number of requested positions %d does not agree with "
                         "the number of servos instantiated %d.",
                         num_incoming_pos, numServos_);
            throw std::runtime_error("PosRequestMismatch");
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    // 2. update the goal position structs
    // 2A. should we check for ID agreement?
    //  for now I will just code it in very dumbly and expect the user to
    //  provide agreement
    for (int i = 0; i < numServos_; i++) {
        try {
            if (servoList_[i].motorId_ == msg.pos_array[i].id) {
                servoList_[i].goalPos_ = msg.pos_array[i].position;
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Incoming servo message at index 0x%2X requests "
                             "MotorID 0x%2X, which is different from servolist "
                             "MotorID 0x%2X at the same index",
                             i, msg.pos_array[i].id, servoList_[i].motorId_);
                throw std::runtime_error("ServoIDMismatch");
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
}

void DynamixelDriver::motorVelGoalCB_(const DynaVelArr& msg) {
    // 1. check if the number of servo positions received is the same as the
    // number of servos loaded from config

    int num_incoming_pos = msg.vel_array.size();
    try {
        if (num_incoming_pos != numMotors_) {
            RCLCPP_ERROR(this->get_logger(),
                         "Number of requested positions %d does not agree with "
                         "the number of servos instantiated %d.",
                         num_incoming_pos, numMotors_);
            throw std::runtime_error("PosRequestMismatch");
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    // 2. update the goal position structs
    // 2A. should we check for ID agreement?
    //  for now I will just code it in very dumbly and expect the user to
    //  provide agreement
    for (int i = 0; i < numMotors_; i++) {
        try {
            if (motorList_[i].motorId_ == msg.vel_array[i].id) {
                motorList_[i].goalVel_ = msg.vel_array[i].velocity;
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Incoming servo message at index 0x%2X requests "
                             "MotorID 0x%2X, which is different from servolist "
                             "MotorID 0x%2X at the same index",
                             i, msg.vel_array[i].id, motorList_[i].motorId_);
                throw std::runtime_error("ServoIDMismatch");
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
}

void DynamixelDriver::writeParams(std::map<uint8_t, uint8_t*> val_map,
                                  uint16_t insturction_code,
                                  uint16_t instruction_len) {
    groupBulkWrite_->clearParam();
    for (auto const& pair : val_map) {
        groupBulkWrite_->addParam(pair.first, insturction_code, instruction_len,
                                  pair.second);
    }
    groupBulkWrite_->txPacket();
    groupBulkWrite_->clearParam();
}

// void DynamixelDriver::readParams(std::map<uint8_t, uint8_t*> val_map,
//                     uint16_t instruction_code, uint16_t instruction_len){
// groupBulkRead_->clearParam();

//     for(auto const& pair: val_map){
//         groupBulkRead_->addParam(pair.first, instruction_code,
//         instruction_len);
//     }
//     groupBulkRead_->rxPacket();

//     }

void DynamixelDriver::enableTorque(std::vector<uint8_t> ids) {
    for (int i = 0; i < ids.size(); i++) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(
            portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Comm error: %s",
                         packetHandler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Comm error %s",
                         packetHandler_->getRxPacketError(dxl_error));
        } else {
            RCLCPP_INFO(this->get_logger(), "Torque on for ID %2x", ids[i]);
        }
    }
}