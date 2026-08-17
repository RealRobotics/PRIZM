#include <chrono>
#include <prizm_dynamixel/dynamixelDriver.hpp>

// Static variables for communicating with all Dynamixels.
dynamixel::PortHandler* DynamixelDriver::portHandler_ = 0;
dynamixel::PacketHandler* DynamixelDriver::packetHandler_ = 0;
dynamixel::GroupBulkRead* DynamixelDriver::groupBulkRead_ = 0;
dynamixel::GroupBulkWrite* DynamixelDriver::groupBulkWrite_ = 0;

DynamixelDriver::DynamixelDriver() : Node("DynaDriver") {}

DynamixelDriver::DynamixelDriver(std::string& nodeName,
                                 rclcpp::NodeOptions& options)
    : Node(nodeName, options) {
    this->declare_parameter(paramNamespace_ + ".debug", true);
    debug = this->get_parameter(paramNamespace_ + ".debug").as_bool();

    this->declare_parameter(paramNamespace_ + ".device_addr", "/dev/ttyUSBX");
    address = this->get_parameter(paramNamespace_ + ".device_addr").as_string();

    auto servo_parameter_map = std::map<std::string, int>{
        {"num_servos", 1},
    };

    this->declare_parameters(paramNamespace_, servo_parameter_map);
    numServos_ = this->get_parameter(paramNamespace_ + ".num_servos").as_int();
    // servoMin_ = this->get_parameter(paramNamespace_ + ".servo_min").as_int();
    // servoMax_ = this->get_parameter(paramNamespace_ + ".servo_max").as_int();

    // this->declare_parameter(paramNamespace_ + ".servoMins",
    // std::vector<int64_t>());
    auto servo_limits_map = std::map<std::string, std::vector<int64_t>>{
        {"servo_mins", std::vector<int64_t>()},
        {"servo_maxs", std::vector<int64_t>()},
        {"servo_zeros", std::vector<int64_t>()},
    };

    this->declare_parameters(paramNamespace_, servo_limits_map);
    servoMins_ =
        this->get_parameter(paramNamespace_ + ".servo_mins").as_integer_array();
    servoMaxs_ =
        this->get_parameter(paramNamespace_ + ".servo_maxs").as_integer_array();
    servoZeros_ = this->get_parameter(paramNamespace_ + ".servo_zeros")
                      .as_integer_array();

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

    auto profile_vals_map = std::map<std::string, float>{
        {"profile_velocity", 0}, {"profile_acceleration", 0}};
    this->declare_parameters(paramNamespace_, profile_vals_map);
    profile_velocity =
        this->get_parameter(paramNamespace_ + ".profile_velocity").as_double();
    profile_acceleration =
        this->get_parameter(paramNamespace_ + ".profile_acceleration")
            .as_double();

    servoIds_.insert(servoIds_.end(), servoIdsINT_.begin(), servoIdsINT_.end());
    motorIds_.insert(motorIds_.end(), motorIdsINT_.begin(), motorIdsINT_.end());

    allIds_.insert(allIds_.end(), servoIdsINT_.begin(), servoIdsINT_.end());
    allIds_.insert(allIds_.end(), motorIdsINT_.begin(), motorIdsINT_.end());

    InitGoalSubscribers();
    InitServices();
    InitDynamixelComms();
    InitMotors();
}

void DynamixelDriver::InitMotors() {
    // check that we have as many IDs as we have motors
    auto op_mode_map = std::map<uint8_t, uint8_t>();

    try {
        if (numServos_ != (int)servoIdsINT_.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Mismatch in number of servos: %d and number of servo ids: %d",
                numServos_, (int)servoIdsINT_.size());
            throw std::runtime_error("ServoNumMismatch");
        } else if (numServos_ != (int)servoMins_.size() ||
                   numServos_ != (int)servoMaxs_.size() ||
                   numServos_ != (int)servoZeros_.size()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Mismatch in number of servos: %d and number of "
                         "limits: Min %d Max %d Zero %d",
                         numServos_, (int)servoMins_.size(),
                         (int)servoMaxs_.size(), (int)servoZeros_.size());
            throw std::runtime_error("ServoLimitsNumMismatch");
        } else {
            dynamixelData servo_default;
            servo_default.operatingMode_ =
                OperatingModes::EXTENDED_POSITION_CONTROL;
            for (int i = 0; i < numServos_; i++) {
                servo_default.servoMin = servoMins_.at(i);
                servo_default.servoMax = servoMaxs_.at(i);
                servo_default.servoZero = servoZeros_.at(i);
                servoMap_.insert({{servoIdsINT_[i], servo_default}});
                op_mode_map.insert(
                    {servoIdsINT_[i], servo_default.operatingMode_});
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
                op_mode_map.insert(
                    {motorIdsINT_[i], motor_default.operatingMode_});
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
    // summary of instanatiation
    RCLCPP_INFO(this->get_logger(),
                "I have instantiated %d servos and %d motors.", numServos_,
                numMotors_);

    // Set the profile velocity for the servos
    auto profile_vel_map = std::map<uint8_t, float>();
    for (auto& pair : servoMap_) {
        profile_vel_map.insert({pair.first, profile_velocity});
        pair.second.profVel_ = profile_velocity;
    }

    // Set the profile acceleration for the motors
    auto profile_accel_map = std::map<uint8_t, float>();
    for (auto& pair : motorMap_) {
        profile_accel_map.insert({pair.first, profile_acceleration});
        pair.second.profAccel_ = profile_acceleration;
    }
    setProfileVelocity(profile_vel_map);
    setProfileAccel(profile_accel_map);

    changeOperatingMode(op_mode_map);
    set_leds(allIds_, 0x01);
}

void DynamixelDriver::InitDynamixelComms() {
    if (!debug) {
        // set up porthandler
        portHandler_ = dynamixel::PortHandler::getPortHandler(address.c_str());
        // set up packethandler
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

        // set up bulkreader
        groupBulkRead_ =
            new dynamixel::GroupBulkRead(portHandler_, packetHandler_);

        // setup bulkwriter
        groupBulkWrite_ =
            new dynamixel::GroupBulkWrite(portHandler_, packetHandler_);

        bool dx1_comm_result = portHandler_->openPort();
        if (!dx1_comm_result) {
            RCLCPP_ERROR(this->get_logger(), "Could not open the port");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("single_servo"),
                        "Port opened successfully");
        }
        dx1_comm_result = portHandler_->setBaudRate(57600);
        if (!dx1_comm_result) {
            RCLCPP_ERROR(this->get_logger(), "Could not set baud rate");
        } else {
            RCLCPP_INFO(this->get_logger(), "Baud rate set successfully");
        }
    }
}

void DynamixelDriver::InitGoalSubscribers() {
    if (numServos_ != 0) {
        servoGoalPosSub_ = this->create_subscription<DynaPosArr>(
            paramNamespace_ + "/servo_pos", 10,
            std::bind(&DynamixelDriver::servoPosGoalCB_, this,
                      std::placeholders::_1));
    }

    if (numMotors_ != 0) {
        motorGoalPosSub_ = this->create_subscription<DynaVelArr>(
            paramNamespace_ + "/motor_vel", 10,
            std::bind(&DynamixelDriver::motorVelGoalCB_, this,
                      std::placeholders::_1));
    }
}

void DynamixelDriver::InitServices() {
    setProfVelSrv_ = this->create_service<DynaSetProfVel>(
        paramNamespace_ + "/set_profile_velocity",
        std::bind(&DynamixelDriver::setProfVelSrvCB_, this,
                  std::placeholders::_1, std::placeholders::_2));

    getStatusSrv_ = this->create_service<DynaGetStatus>(
        paramNamespace_ + "/get_status",
        std::bind(&DynamixelDriver::getStatusCB_, this, std::placeholders::_1,
                  std::placeholders::_2));

    setTorqueSrv_ = this->create_service<SetBool>(
        paramNamespace_ + "/set_torque",
        std::bind(&DynamixelDriver::setTorqueCB_, this, std::placeholders::_1,
                  std::placeholders::_2));
}

void DynamixelDriver::readMotorID() {
    for (const auto& pair : servoMap_) {
        auto id = pair.first;
        uint8_t id_read;
        dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, 7,
                                                        &id_read, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(),
                         "read for id %d did not succeed. Result %s", id,
                         packetHandler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "id: %d Received error: %s", id,
                         packetHandler_->getRxPacketError(dxl_error));
        }
        RCLCPP_INFO(this->get_logger(),
                    "id: %d, I have read the id of the servo is: %d", id,
                    id_read);
    }
}

void DynamixelDriver::readMotorLED() {
    for (const auto& pair : servoMap_) {
        auto id = pair.first;
        uint8_t led_read = 0xFF;
        dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, 65,
                                                        &led_read, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(),
                         "read for id %d did not succeed. Result %s", id,
                         packetHandler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "id: %d Received error: %s", id,
                         packetHandler_->getRxPacketError(dxl_error));
        }
        RCLCPP_INFO(this->get_logger(),
                    "id: %d, I have read the LED of the servo is: %d", id,
                    led_read);
    }
}

void DynamixelDriver::servoPosGoalCB_(const DynaPosArr& msg) {
    std::map<uint8_t, float> goal_map;

    int num_incoming_pos = msg.pos_array.size();
    RCLCPP_INFO(this->get_logger(), "Received a request for %d positions",
                num_incoming_pos);
    for (int i = 0; i < num_incoming_pos; i++) {
        try {
            auto find_it = servoMap_.find(msg.pos_array[i].id);
            if (find_it == servoMap_.end()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Incoming message requests servo id 0x%02x, which "
                             "is not in the list of valid ids",
                             msg.pos_array[i].id);
                throw std::runtime_error("ServoIDMismatch");
            } else {
                if (msg.pos_array[i].position > find_it->second.servoMax ||
                    msg.pos_array[i].position < find_it->second.servoMin) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Requested position %f for servo id %d, which "
                                 "is outside of bounds Min %f Max %f",
                                 msg.pos_array[i].position, msg.pos_array[i].id,
                                 find_it->second.servoMin,
                                 find_it->second.servoMax);
                    continue;
                } else {
                    find_it->second.goalPos_ = msg.pos_array[i].position;
                    goal_map.insert({find_it->first, find_it->second.goalPos_});
                }
            }

        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
    if (!debug) setServoPos(goal_map);
}

void DynamixelDriver::motorVelGoalCB_(const DynaVelArr& msg) {
    std::map<uint8_t, float> prof_vel_map, goal_vel_map;

    int num_incoming_vel = msg.vel_array.size();
    for (int i = 0; i < num_incoming_vel; i++) {
        try {
            auto motor_find_it = motorMap_.find(msg.vel_array[i].id);
            auto servo_find_it = servoMap_.find(msg.vel_array[i].id);
            if (motor_find_it != motorMap_.end()) {
                motor_find_it->second.goalVel_ = msg.vel_array[i].velocity;
                goal_vel_map.insert(
                    {motor_find_it->first, motor_find_it->second.goalVel_});
            }
            if (servo_find_it != servoMap_.end()) {
                servo_find_it->second.profVel_ = msg.vel_array[i].velocity;
                prof_vel_map.insert(
                    {servo_find_it->first, servo_find_it->second.profVel_});
            }
            if (!goal_vel_map.empty()) setGoalVelocity(goal_vel_map);
            if (!prof_vel_map.empty()) setProfileVelocity(prof_vel_map);

        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
    singleRead4B(0x00, GOAL_VEL_ADDR);
    singleRead4B(0x01, GOAL_VEL_ADDR);
}

void DynamixelDriver::setProfVelSrvCB_(
    const DynaSetProfVel::Request::SharedPtr req,
    DynaSetProfVel::Response::SharedPtr res) {
    // DynaVelArr vel_arr = req->vel_array;
    std::map<uint8_t, float> vel_map;
    for (int i = 0; i < (int)req->vel_array.size(); i++) {
        vel_map.insert({req->vel_array[i].id, req->vel_array[i].velocity});
    }
    setProfileVelocity(vel_map);
    res->success = true;
}

void DynamixelDriver::setTorqueCB_(const SetBool::Request::SharedPtr req,
                                   SetBool::Response::SharedPtr res) {
    set_torque(allIds_, req->data);
    res->success = true;
    res->message = "Successfully toggled torque";
}

void DynamixelDriver::getStatusCB_(const DynaGetStatus::Request::SharedPtr req,
                                   DynaGetStatus::Response::SharedPtr res) {
    DynaPosArr pos_arr;
    DynaVelArr vel_arr;
    int num_ids = req->id.size();
    pos_arr.pos_array.resize(num_ids);
    vel_arr.vel_array.resize(num_ids);
    res->operating_mode.resize(num_ids);
    res->torque_enable.resize(num_ids);
    // res->toggle.resize(num_ids);
    auto positions = getPos(req->id);
    auto velocities = getVel(req->id);

    for (int i = 0; i < num_ids; i++) {
        pos_arr.pos_array[i].id = req->id[i];
        pos_arr.pos_array[i].position = positions[req->id[i]];

        vel_arr.vel_array[i].id = req->id[i];
        vel_arr.vel_array[i].velocity = velocities[req->id[i]];
        res->operating_mode[i] = singleRead(req->id[i], OP_MODE_ADDR);
        res->torque_enable[i] = singleRead(req->id[i], TORQUE_ENABLE_ADDR);
    }
    res->pos_array = pos_arr;
    res->vel_array = vel_arr;
}

int DynamixelDriver::writeParams(
    std::map<uint8_t, std::vector<uint8_t>> val_map, uint16_t insturction_code,
    uint16_t instruction_len) {
    std::scoped_lock lock(comms_mutex_);
    groupBulkWrite_->clearParam();

    for (auto const& pair : val_map) {
        uint8_t hex[4];
        hex[0] = pair.second[0];
        hex[1] = pair.second[1];
        hex[2] = pair.second[2];
        hex[3] = pair.second[3];

        auto dxl_addparam_result = groupBulkWrite_->addParam(
            pair.first, insturction_code, instruction_len, hex);
        if (!dxl_addparam_result) {
            RCLCPP_ERROR(this->get_logger(),
                         "[ID: %d groupBulkWrite addparam failed",
                         (int)pair.first);
            return 0;
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Bulk Write: Added param ID: 0x%x, Val: 0x%x%x%x%x",
                        pair.first, hex[3], hex[2], hex[1], hex[0]);
        }
    }
    dxl_comm_result = groupBulkWrite_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), "Had an error %s in writing instruction %d",
            packetHandler_->getTxRxResult(dxl_comm_result), insturction_code);
    }
    groupBulkWrite_->clearParam();
    switch (dxl_comm_result) {
        case COMM_SUCCESS:
            RCLCPP_INFO(this->get_logger(), "Successfully wrote instruction %d",
                        insturction_code);
            return 1;
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Failed to write instruction");
            return -1;
            break;
    }
}

std::map<uint8_t, uint32_t> DynamixelDriver::readParams(
    std::vector<uint8_t> addresses, uint16_t instruction_code,
    uint16_t instruction_len) {
    std::scoped_lock lock(comms_mutex_);
    std::map<uint8_t, uint32_t> res_map;
    groupBulkRead_->clearParam();
    for (auto& i : addresses) {
        auto dxl_addparam_result =
            groupBulkRead_->addParam(i, instruction_code, instruction_len);
        if (!dxl_addparam_result) {
            RCLCPP_ERROR(this->get_logger(),
                         "[ID: %d groupBulkRead addparam failed", i);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Bulk Read: Added param ID: 0x%x for code %d", i,
                        instruction_code);
        }
    }

    dxl_comm_result = groupBulkRead_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to receive data from bulk read: %s",
                     packetHandler_->getTxRxResult(dxl_comm_result));
    } else {
        int dxl_getdata_res = false;
        // wait for the data to be available
        for (auto& i : addresses) {
            dxl_getdata_res = groupBulkRead_->isAvailable(i, instruction_code,
                                                          instruction_len);
            if (dxl_getdata_res) {
                // RCLCPP_INFO(this->get_logger(),
                //             "Data is available, reading now");
                auto data = groupBulkRead_->getData(i, instruction_code,
                                                    instruction_len);
                res_map.insert({i, data});
            } else {
                RCLCPP_INFO(this->get_logger(), "Data is not avaialable");
            }
        }
    }
    return res_map;
}
void DynamixelDriver::enableTorque(std::vector<uint8_t> ids) {
    std::scoped_lock lock(comms_mutex_);

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
        if (!debug) {
            dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, ids[i], TORQUE_ENABLE_ADDR, 0x01, &dxl_error);

            dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, ids[i], TORQUE_ENABLE_ADDR, 0x01, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error: %s",
                             ids[i],
                             packetHandler_->getTxRxResult(dxl_comm_result));
            } else if (dxl_error != 0) {
                RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error %s",
                             ids[i],
                             packetHandler_->getRxPacketError(dxl_error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Torque on for ID %2x", ids[i]);
            }
        }
    }
}

void DynamixelDriver::set_torque(std::vector<uint8_t> ids, uint8_t toggle) {
    std::scoped_lock lock(comms_mutex_);

    for (size_t i = 0; i < ids.size(); i++) {
        auto servoMap_it = servoMap_.find(ids[i]);
        auto motorMap_it = motorMap_.find(ids[i]);

        if (servoMap_it != servoMap_.end()) {
            servoMap_it->second.torqueEnable_ = toggle;
            if (!debug) {
                dxl_comm_result = packetHandler_->write1ByteTxRx(
                    portHandler_, ids[i], TORQUE_ENABLE_ADDR, toggle,
                    &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(
                        this->get_logger(), "ID: 0x%2X Comm error: %s", ids[i],
                        packetHandler_->getTxRxResult(dxl_comm_result));
                } else if (dxl_error != 0) {
                    RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error %s ",
                                 ids[i],
                                 packetHandler_->getRxPacketError(dxl_error));
                }
            }
        } else if (motorMap_it != motorMap_.end()) {
            if (!debug) {
                dxl_comm_result = packetHandler_->write1ByteTxRx(
                    portHandler_, ids[i], TORQUE_ENABLE_ADDR, toggle,
                    &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(
                        this->get_logger(), "ID: 0x%2X Comm error: %s", ids[i],
                        packetHandler_->getTxRxResult(dxl_comm_result));
                } else if (dxl_error != 0) {
                    RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error %s ",
                                 ids[i],
                                 packetHandler_->getRxPacketError(dxl_error));
                }
            }
            motorMap_it->second.torqueEnable_ = toggle;
        }
    }
}

void DynamixelDriver::set_leds(std::vector<uint8_t> ids, uint8_t toggle) {
    // std::scoped_lock lock(comms_lock_);

    for (size_t i = 0; i < ids.size(); i++) {
        auto servoMap_it = servoMap_.find(ids[i]);
        auto motorMap_it = motorMap_.find(ids[i]);

        if (servoMap_it != servoMap_.end()) {
            if (!debug) {
                dxl_comm_result = packetHandler_->write1ByteTxRx(
                    portHandler_, ids[i], LED_ENABLE_ADDR, toggle, &dxl_error);
            }
        } else if (motorMap_it != motorMap_.end()) {
            if (!debug) {
                dxl_comm_result = packetHandler_->write1ByteTxRx(
                    portHandler_, ids[i], LED_ENABLE_ADDR, toggle, &dxl_error);
            }
        }
    }
}

void DynamixelDriver::changeOperatingMode(
    std::map<uint8_t, uint8_t> op_mode_map) {
    set_torque(allIds_, 0x00);

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
        if (!debug) {
            dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, pair.first, OP_MODE_ADDR, pair.second,
                &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to set Control Mode ID: 0x%x, Value: 0x%x.",
                    pair.first, pair.second);
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Succeeded to set Control Mode ID: 0x%x, "
                            "Value: 0x%x.",
                            pair.first, pair.second);
            }
        }
    }
    set_torque(allIds_, 0x01);
}

void DynamixelDriver::setProfileVelocity(std::map<uint8_t, float> vel_map) {
    // set_torque(motorIds_, false);
    std::map<uint8_t, std::vector<uint8_t>> map;
    std::vector<uint8_t> vel_arr(4);
    int write_result = 1;
    for (auto const pair : vel_map) {
        // 1. update the cached values
        auto id = pair.first;
        auto vel = pair.second;
        // auto accel = pair.second[1];

        if (servoMap_.find(id) != servoMap_.end()) {
            servoMap_[id].goalVel_ = vel;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Servo was not in address list");
            continue;
        }
        // 2. build the instruction map for write params
        vel_arr = rpmToHex(vel);
        map.insert({id, vel_arr});
    }
    if (!debug) write_result = writeParams(map, PROF_VEL_ADDR, PROF_VEL_LEN);

    if (write_result == 1) {
        for (auto const pair : vel_map) {
            auto id = pair.first;
            auto pos = pair.second;
            if (servoMap_.find(id) != servoMap_.end()) {
                servoMap_[id].profVel_ = pos;
            }
        }
    }
}

void DynamixelDriver::setProfileAccel(std::map<uint8_t, float> accel_map) {
    BulkParamMap map;
    std::vector<uint8_t> accel_arr(4);
    int write_result = 1;
    for (auto const pair : accel_map) {
        // 1. update the cached values
        auto id = pair.first;
        auto accel = pair.second;
        if (motorMap_.find(id) != motorMap_.end()) {
            motorMap_[id].profAccel_ = accel;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Servo was not in address list");
            continue;
        }
        // 2. build the instruction map for write params
        accel_arr = accelToHex(accel);
        map.insert({id, accel_arr});
    }
    if (!debug) write_result = writeParams(map, PROF_VEL_ADDR, PROF_VEL_LEN);

    if (write_result == 1) {
        for (auto const pair : accel_map) {
            auto id = pair.first;
            auto pos = pair.second;
            if (motorMap_.find(id) != motorMap_.end()) {
                motorMap_[id].profAccel_ = pos;
            }
        }
    }
}

void DynamixelDriver::setGoalVelocity(std::map<uint8_t, float> vel_map) {
    // set_torque(motorIds_, false);
    std::map<uint8_t, std::vector<uint8_t>> map;
    std::vector<uint8_t> vel_arr(4);
    int write_result = 1;
    for (auto const pair : vel_map) {
        // 1. update the cached values
        auto id = pair.first;
        auto vel = pair.second;
        RCLCPP_INFO(this->get_logger(),
                    "Populating vel_map for goal velocity of id %d", id);

        if (motorMap_.find(id) != motorMap_.end()) {
            motorMap_[id].goalVel_ = vel;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Motor was not in address list");
            continue;
        }

        // 2. build the instruction map for write params
        vel_arr = rpmToHex(vel);
        map.insert({id, vel_arr});
    }
    if (!debug) write_result = writeParams(map, GOAL_VEL_ADDR, GOAL_VEL_LEN);

    if (write_result == 1) {
        for (auto const pair : vel_map) {
            auto id = pair.first;
            auto vel = pair.second;

            if (motorMap_.find(id) != motorMap_.end()) {
                motorMap_[id].currentVel_ = vel;
            }
        }
    }
}

void DynamixelDriver::setServoPos(std::map<uint8_t, float> pos_map) {
    std::map<uint8_t, std::vector<uint8_t>> map;

    std::vector<uint8_t> pos_arr(4);
    int write_result = 1;
    for (auto const pair : pos_map) {
        auto id = pair.first;
        auto pos = pair.second;

        if (servoMap_.find(id) != servoMap_.end()) {
            servoMap_[id].goalPos_ = pos;
            pos_arr = degToPos(pos);
            map.insert({id, pos_arr});
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Servo ID: 0x%x is not in the list of "
                         "instantiated servos",
                         id);
        }
    }
    if (!debug && !map.empty())
        write_result = writeParams(map, GOAL_POS_ADDR, GOAL_POS_LEN);

    if (write_result == 1 && !map.empty()) {
        for (auto const pair : pos_map) {
            auto id = pair.first;
            auto pos = pair.second;
            if (servoMap_.find(id) != servoMap_.end()) {
                servoMap_[id].currentPos_ = pos;
            }
        }
        singleRead4B(0x00, 132);
        singleRead4B(0x01, 132);
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Something went wrong with teh write params. Could not "
                     "set servo pos");
    }
}

std::map<uint8_t, float> DynamixelDriver::getPos(std::vector<uint8_t> ids) {
    auto hex_map = readParams(ids, CURR_POS_ADDR, CURR_POS_LEN);
    std::map<uint8_t, float> float_map;
    for (auto const pair : hex_map) {
        float_map.insert({pair.first, posToDeg(pair.second)});
    }
    return float_map;
}

std::map<uint8_t, float> DynamixelDriver::getVel(std::vector<uint8_t> ids) {
    auto hex_map = readParams(ids, CURR_VEL_ADDR, CURR_VEL_LEN);
    std::map<uint8_t, float> float_map;
    for (auto const pair : hex_map) {
        float_map.insert({pair.first, hexToRpm(pair.second)});
    }
    return float_map;
}

uint8_t DynamixelDriver::singleRead(uint8_t id, uint16_t code) {
    std::scoped_lock lock(comms_mutex_);
    uint8_t output = 0;
    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, code,
                                                    &output, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(),
                     "Read for id %d of code %d was not successful. %s", id,
                     code, packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Read for ID %d of code %d resulted in an error. %s", id,
                     code, packetHandler_->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "Single Read from id %d, code %d obtained: %x", id, code,
                    output);
    }
    return output;
}

void DynamixelDriver::singleRead4B(uint8_t id, uint16_t code) {
    uint32_t output = 0;
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, code,
                                                    &output, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(),
                     "Read for id %d of code %d was not successful. %s", id,
                     code, packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Read for ID %d of code %d resulted in an error. %s", id,
                     code, packetHandler_->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(this->get_logger(), "Read from id %d, code %d obtained: %d",
                    id, code, output);
    }
}

void DynamixelDriver::singleWrite(uint8_t id, uint16_t code, uint8_t data) {
    dxl_comm_result = packetHandler_->write1ByteTxRx(
        portHandler_, id, code, uint32_t(data), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(),
            "writing instruction %d to id %d rsulted in comm result %s", code,
            id, packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        // std::cout << packetHandler_->getRxPacketError(dxl_error) <<
        // std::endl;
        RCLCPP_ERROR(
            this->get_logger(),
            "writing instruction %d to id %d rsulted in comm result %s", code,
            id, packetHandler_->getRxPacketError(dxl_error));
    }
    RCLCPP_INFO(this->get_logger(), "wrote position %x to servo id %d", data,
                id);
}

void DynamixelDriver::singleWrite4B(uint8_t id, uint16_t code, uint32_t data) {
    dxl_comm_result = packetHandler_->write4ByteTxRx(
        portHandler_, id, code, uint32_t(data), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(),
            "writing instruction %d to id %d rsulted in comm result %s", code,
            id, packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        // std::cout << packetHandler_->getRxPacketError(dxl_error) <<
        // std::endl;
        RCLCPP_ERROR(
            this->get_logger(),
            "writing instruction %d to id %d rsulted in comm result %s", code,
            id, packetHandler_->getRxPacketError(dxl_error));
    }
    RCLCPP_INFO(this->get_logger(), "wrote position %x to servo id %d", data,
                id);
}

DynamixelDriver::~DynamixelDriver() {
    set_torque(allIds_, 0x00);
    set_leds(allIds_, 0x00);
    if (!debug) portHandler_->closePort();
}
