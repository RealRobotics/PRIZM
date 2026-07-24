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

    servoIds_.insert(servoIds_.end(), servoIdsINT_.begin(), servoIdsINT_.end());
    motorIds_.insert(motorIds_.end(), motorIdsINT_.begin(), motorIdsINT_.end());

    allIds_.insert(allIds_.end(), servoIdsINT_.begin(), servoIdsINT_.end());
    allIds_.insert(allIds_.end(), motorIdsINT_.begin(), motorIdsINT_.end());

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
            dynamixelData servo_default;
            servo_default.operatingMode_ =
                OperatingModes::EXTENDED_POSITION_CONTROL;
            for (int i = 0; i < numServos_; i++) {
                servoMap_.insert({{servoIdsINT_[i], servo_default}});
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
            dynamixelData motor_default;
            motor_default.operatingMode_ = OperatingModes::VELOCITY_CONTROL;
            for (int i = 0; i < numMotors_; i++) {
                motorMap_.insert({motorIdsINT_[i], motor_default});
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    // Enable Torque for all motors
}

void DynamixelDriver::InitDynamixelComms() {
    std::scoped_lock lock(comms_lock_);

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

    // std::vector<uint8_t, float> goal_map;
    uint8_t goal_arr[4];
    BulkParamMap map;
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
            auto find_it = servoMap_.find(msg.pos_array[i].id);
            if (find_it == servoMap_.end()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Incoming message requests servo id 0x%02x, which "
                             "is not in the list of valid ids",
                             msg.pos_array[i].id);
                throw std::runtime_error("ServoIDMismatch");
            } else {
                find_it->second.goalPos_ = msg.pos_array[i].position;
                degToPos(find_it->second.goalPos_, goal_arr);
                map.insert({find_it->first, goal_arr});
                writeParams(map, GOAL_POS_ADDR, GOAL_POS_LEN);
            }

        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
}

void DynamixelDriver::motorVelGoalCB_(const DynaVelArr& msg) {
    // 1. check if the number of servo positions received is the same as the
    // number of servos loaded from config

    std::vector<uint8_t, float> vel_map;
    uint8_t vel_arr[4];
    BulkParamMap map;

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
            auto find_it = motorMap_.find(msg.vel_array[i].id);
            if (find_it == motorMap_.end()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Incoming message requests motor id 0x%02x, which "
                             "is not in the list of valid ids",
                             msg.vel_array[i].id);
                throw std::runtime_error("MotorIDMismatch");
            } else {
                find_it->second.goalVel_ = msg.vel_array[i].velocity;
            }
            rpmToHex(find_it->second.goalVel_, vel_arr);
            map.insert({find_it->first, vel_arr});
            
            writeParams(map, GOAL_VEL_ADDR, GOAL_VEL_LEN);

        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
}

void DynamixelDriver::writeParams(std::map<uint8_t, uint8_t*> val_map,
                                  uint16_t insturction_code,
                                  uint16_t instruction_len) {
    std::scoped_lock lock(comms_lock_);
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
    // std::scoped_lock lock(comms_lock_);

    for (size_t i = 0; i < ids.size(); i++) {
        auto servoMap_it = servoMap_.find(ids[i]);
        auto motorMap_it = motorMap_.find(ids[i]);

        if (servoMap_it != servoMap_.end()) {
            // dxl_comm_result = packetHandler_->write1ByteTxRx(
            //     portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
            servoMap_it->second.torqueEnable_ = true;
        } else if (motorMap_it != motorMap_.end()) {
            // dxl_comm_result = packetHandler_->write1ByteTxRx(
            //     portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
            motorMap_it->second.torqueEnable_ = true;
        }
        // dxl_comm_result = packetHandler_->write1ByteTxRx(
        //     portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);

        //     dxl_comm_result = packetHandler_->write1ByteTxRx(
        //         portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
        //     if (dxl_comm_result != COMM_SUCCESS) {
        //         RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error: %s",
        //         ids[i],
        //                      packetHandler_->getTxRxResult(dxl_comm_result));
        //     } else if (dxl_error != 0) {
        //         RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error %s",
        //         ids[i],
        //                      packetHandler_->getRxPacketError(dxl_error));
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "Torque on for ID %2x",
        //         ids[i]);
        //     }
    }
}

void DynamixelDriver::toggle_torque(std::vector<uint8_t> ids, bool toggle) {
    // std::scoped_lock lock(comms_lock_);

    for (size_t i = 0; i < ids.size(); i++) {
        auto servoMap_it = servoMap_.find(ids[i]);
        auto motorMap_it = motorMap_.find(ids[i]);

        if (servoMap_it != servoMap_.end()) {
            // dxl_comm_result = packetHandler_->write1ByteTxRx(
            //     portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
            servoMap_it->second.torqueEnable_ = toggle;
        } else if (motorMap_it != motorMap_.end()) {
            // dxl_comm_result = packetHandler_->write1ByteTxRx(
            //     portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
            motorMap_it->second.torqueEnable_ = toggle;
        }
        // dxl_comm_result = packetHandler_->write1ByteTxRx(
        //     portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);

        //     dxl_comm_result = packetHandler_->write1ByteTxRx(
        //         portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
        //     if (dxl_comm_result != COMM_SUCCESS) {
        //         RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error: %s",
        //         ids[i],
        //                      packetHandler_->getTxRxResult(dxl_comm_result));
        //     } else if (dxl_error != 0) {
        //         RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error %s",
        //         ids[i],
        //                      packetHandler_->getRxPacketError(dxl_error));
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "Torque on for ID %2x",
        //         ids[i]);
        //     }
    }
}

void DynamixelDriver::changeOperatingMode(
    std::map<uint8_t, uint8_t> op_mode_map) {
    std::scoped_lock lock(comms_lock_);

    toggle_torque(allIds_, false);

    // iterate over the map
    for (auto const pair : op_mode_map) {
        // 1. validate that the id is present
        auto id = pair.first;
        auto servo_it = servoMap_.find(id);
        auto motor_it = motorMap_.find(id);

        // 2. update the struct data
        if (servo_it != servoMap_.end()) {
            servo_it->second.operatingMode_ = pair.second;
        } else if (motor_it != motorMap_.end()) {
            motor_it->second.operatingMode_ = pair.second;
        }

        // 3. then write it to the servo
        // dxl_comm_result = packetHandler_->write1ByteTxRx(
        //     portHandler_, pair.first, OP_MODE_ADDR, dxl_error);
    }
    toggle_torque(allIds_, true);
}

void DynamixelDriver::setVelocity(std::map<uint8_t, float> vel_map) {
    toggle_torque(motorIds_, false);
    BulkParamMap map;
    uint8_t vel_arr[4];

    for (auto const pair : vel_map) {
        // 1. update the cached values
        auto id = pair.first;
        auto vel = pair.second;
        // auto accel = pair.second[1];

        if (motorMap_.find(id) != motorMap_.end()) {
            motorMap_[id].goalVel_ = vel;
            // motorMap_[id].goalAccel_ = accel;
        }
        // 2. build the instruction map for write params
        rpmToHex(vel, vel_arr);
        map.insert({id, vel_arr});
    }
    // writeParams(map, PROF_VEL_ADDR, PROF_VEL_LEN);
    toggle_torque(motorIds_, true);
}