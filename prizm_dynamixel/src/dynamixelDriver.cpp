#include <prizm_dynamixel/dynamixelDriver.hpp>

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

    InitDynamixelComms();
    InitMotors();
    InitGoalSubscribers();
    InitServices();
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
        } else {
            dynamixelData servo_default;
            servo_default.operatingMode_ =
                OperatingModes::EXTENDED_POSITION_CONTROL;
            for (int i = 0; i < numServos_; i++) {
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

    // Enable Torque for all motors
    // enableTorque(allIds_);

    // Set the profile velocity for the servos
    auto profile_vel_map = std::map<uint8_t, float>();
    for (auto& pair : servoMap_) {
        profile_vel_map.insert({pair.first, profile_velocity});
        pair.second.profVel_ = profile_velocity;
    }
    // setProfileVelocity(profile_vel_map);

    // Set the profile acceleration for the motors
    auto profile_accel_map = std::map<uint8_t, float>();
    for (auto& pair : motorMap_) {
        profile_accel_map.insert({pair.first, profile_acceleration});
        pair.second.profAccel_ = profile_acceleration;
    }
    // setProfileAccel(profile_accel_map);
    changeOperatingMode(op_mode_map);
}

void DynamixelDriver::InitDynamixelComms() {
    if (!debug) {
        // set up porthandler
        portHandler_ = dynamixel::PortHandler::getPortHandler(address.c_str());
        // set up packethandler
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

        // set up bulkreader
        groupBulkRead_ = std::make_shared<dynamixel::GroupBulkRead>(
            portHandler_, packetHandler_);

        // setup bulkwriter
        groupBulkWrite_ = std::make_shared<dynamixel::GroupBulkWrite>(
            portHandler_, packetHandler_);

        if (portHandler_->openPort()) {
            std::cout << "Succeeded to open the port!\n";
        } else {
            std::cout << "Failed to open the port!\n";
        }

        if (portHandler_->setBaudRate(57600)) {
            std::cout << "Succeeded to change the baudrate!\n";
        } else {
            std::cout << "Failed to change the baudrate!\n";
        }
    }
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

void DynamixelDriver::InitServices() {
    RCLCPP_INFO(this->get_logger(), "Starting services now");
    setProfVelSrv_ = this->create_service<DynaSetProfVel>(
        "set_profile_velocity",
        std::bind(&DynamixelDriver::setProfVelSrvCB_, this,
                  std::placeholders::_1, std::placeholders::_2));

    getStatusSrv_ = this->create_service<DynaGetStatus>(
        "get_status", std::bind(&DynamixelDriver::getStatusCB_, this,
                                std::placeholders::_1, std::placeholders::_2));

    setTorqueSrv_ = this->create_service<SetBool>(
        "toggle_torque",
        std::bind(&DynamixelDriver::setTorqueCB_, this, std::placeholders::_1,
                  std::placeholders::_2));
}

void DynamixelDriver::servoPosGoalCB_(const DynaPosArr& msg) {
    // std::vector<uint8_t, float> goal_map;
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
                find_it->second.goalPos_ = msg.pos_array[i].position;
                goal_map.insert({find_it->first, find_it->second.goalPos_});
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
            // } else {
            //     find_it->second.goalVel_ = msg.vel_array[i].velocity;
            //     vel_map.insert({find_it->first, find_it->second.goalVel_});
            // }
            // setGoalVelocity(vel_map);

        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
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
    toggle_torque(allIds_, req->data);
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
    auto positions = getPos(req->id);
    auto velocities = getVel(req->id);

    for (int i = 0; i < num_ids; i++) {
        pos_arr.pos_array[i].id = req->id[i];
        pos_arr.pos_array[i].position = positions[req->id[i]];

        vel_arr.vel_array[i].id = req->id[i];
        vel_arr.vel_array[i].velocity = velocities[req->id[i]];
    }
    res->pos_array = pos_arr;
    res->vel_array = vel_arr;
}

int DynamixelDriver::writeParams(std::map<uint8_t, uint8_t*> val_map,
                                 uint16_t insturction_code,
                                 uint16_t instruction_len) {
    std::scoped_lock lock(comms_mutex_);
    groupBulkWrite_->clearParam();

    // RCLCPP_INFO(this->get_logger(), "0x%x%x%x%x", val_map[0x00][3],
    //             val_map[0x00][2], val_map[0x00][1], val_map[0x00][0]);
    // RCLCPP_INFO(this->get_logger(), "0x%x%x%x%x", val_map[0x01][3],
    //             val_map[0x01][2], val_map[0x01][1], val_map[0x01][0]);

    // for (auto const& pair : val_map) {
    //     RCLCPP_INFO(this->get_logger(), "writeParam: k: 0x%x v: 0x%x%x%x%x",
    //                 pair.first, pair.second[3], pair.second[1],
    //                 pair.second[2], pair.second[0]);

    for (auto const& pair : val_map) {
        auto dxl_addparam_result = groupBulkWrite_->addParam(
            pair.first, insturction_code, instruction_len, pair.second);
        if (!dxl_addparam_result) {
            std::cout << "[ID:" << (int)pair.first
                      << "]groupBulkWrite addparam failed" << std::endl;
            return 0;
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Added param ID: 0x%x, Val: 0x%x%x%x%x", pair.first,
                        pair.second[3], pair.second[2], pair.second[1],
                        pair.second[0]);
        }
    }
    dxl_comm_result = groupBulkWrite_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << packetHandler_->getTxRxResult(dxl_comm_result)
                  << std::endl;
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


int DynamixelDriver::writeParams(std::map<uint8_t, std::vector<uint8_t>> val_map,
                                 uint16_t insturction_code,
                                 uint16_t instruction_len) {
    std::scoped_lock lock(comms_mutex_);
    groupBulkWrite_->clearParam();

    // RCLCPP_INFO(this->get_logger(), "0x%x%x%x%x", val_map[0x00][3],
    //             val_map[0x00][2], val_map[0x00][1], val_map[0x00][0]);
    // RCLCPP_INFO(this->get_logger(), "0x%x%x%x%x", val_map[0x01][3],
    //             val_map[0x01][2], val_map[0x01][1], val_map[0x01][0]);

    // for (auto const& pair : val_map) {
    //     RCLCPP_INFO(this->get_logger(), "writeParam: k: 0x%x v: 0x%x%x%x%x",
    //                 pair.first, pair.second[3], pair.second[1],
    //                 pair.second[2], pair.second[0]);

    for (auto const& pair : val_map) {
        uint8_t hex[4];
        hex[0] = pair.second[0];
        hex[1] = pair.second[1];
        hex[2] = pair.second[2];
        hex[3] = pair.second[3];
        auto dxl_addparam_result = groupBulkWrite_->addParam(
            pair.first, insturction_code, instruction_len, hex);
        if (!dxl_addparam_result) {
            std::cout << "[ID:" << (int)pair.first
                      << "]groupBulkWrite addparam failed" << std::endl;
            return 0;
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Added param ID: 0x%x, Val: 0x%x%x%x%x", pair.first,
                        hex[3], hex[2], hex[1],
                        hex[0]);
        }
    }
    dxl_comm_result = groupBulkWrite_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << packetHandler_->getTxRxResult(dxl_comm_result)
                  << std::endl;
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
    std::map<uint8_t, uint32_t> res_map;
    groupBulkRead_->clearParam();
    for (auto& i : addresses) {
        groupBulkRead_->addParam(i, instruction_code, instruction_len);
    }
    dxl_comm_result = groupBulkRead_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to receive data from bulk read");
    } else {
        int dxl_getdata_res = true;
        // wait for the data to be available
        for (auto& i : addresses) {
            dxl_getdata_res = groupBulkRead_->isAvailable(i, instruction_code,
                                                          instruction_len);
            if (dxl_getdata_res) {
                auto data = groupBulkRead_->getData(i, instruction_code,
                                                    instruction_len);
                res_map.insert({i, data});
            }
        }
    }
    return res_map;
}
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
        if (!debug) {
            dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);

            dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
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

void DynamixelDriver::toggle_torque(std::vector<uint8_t> ids, bool toggle) {
    // std::scoped_lock lock(comms_lock_);

    for (size_t i = 0; i < ids.size(); i++) {
        auto servoMap_it = servoMap_.find(ids[i]);
        auto motorMap_it = motorMap_.find(ids[i]);

        if (servoMap_it != servoMap_.end()) {
            if (!debug) {
                dxl_comm_result = packetHandler_->write1ByteTxRx(
                    portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
            }
            servoMap_it->second.torqueEnable_ = toggle;
        } else if (motorMap_it != motorMap_.end()) {
            if (!debug) {
                dxl_comm_result = packetHandler_->write1ByteTxRx(
                    portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
            }
            motorMap_it->second.torqueEnable_ = toggle;
        }
        if (!debug) {
            dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);

            dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, ids[i], TORQUE_ENABLE_ADDR, dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error: %s",
                             ids[i],
                             packetHandler_->getTxRxResult(dxl_comm_result));
            } else if (dxl_error != 0) {
                RCLCPP_ERROR(this->get_logger(), "ID: 0x%2X Comm error %s",
                             ids[i],
                             packetHandler_->getRxPacketError(dxl_error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Torque %d for ID %2x", toggle,
                            ids[i]);
            }
        }
    }
}

void DynamixelDriver::changeOperatingMode(
    std::map<uint8_t, uint8_t> op_mode_map) {
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
        if (!debug) {
            dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, pair.first, OP_MODE_ADDR, dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to set Control Mode ID: 0x%x, Value: 0x%x.",
                    pair.first, pair.second);
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Succeeded to set Control Mode ID: 0x%x, Value: 0x%x.",
                    pair.first, pair.second);
            }
        }
    }
    toggle_torque(allIds_, true);
}

void DynamixelDriver::setProfileVelocity(std::map<uint8_t, float> vel_map) {
    // toggle_torque(motorIds_, false);
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
    // toggle_torque(motorIds_, true);
}

void DynamixelDriver::setProfileAccel(std::map<uint8_t, float> accel_map) {
    // toggle_torque(motorIds_, false);
    BulkParamMap map;
    uint8_t accel_arr[4];
    int write_result = 1;
    for (auto const pair : accel_map) {
        // 1. update the cached values
        auto id = pair.first;
        auto accel = pair.second;
        // auto accel = pair.second[1];

        if (motorMap_.find(id) != motorMap_.end()) {
            motorMap_[id].profAccel_ = accel;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Servo was not in address list");
            continue;
        }
        // 2. build the instruction map for write params
        accelToHex(accel, accel_arr);
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
    // toggle_torque(motorIds_, true);
}

void DynamixelDriver::setGoalVelocity(std::map<uint8_t, float> vel_map) {
    // toggle_torque(motorIds_, false);
    std::map<uint8_t, std::vector<uint8_t>> map;
    std::vector<uint8_t> vel_arr(4);
    int write_result = 1;

    for (auto const pair : vel_map) {
        // 1. update the cached values
        auto id = pair.first;
        auto vel = pair.second;
        // auto accel = pair.second[1];

        if (motorMap_.find(id) != motorMap_.end()) {
            motorMap_[id].goalVel_ = vel;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Servo was not in address list");
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
    // toggle_torque(servoIds_, false);
    std::map<uint8_t, std::vector<uint8_t>> map;
    
    std::vector<uint8_t> pos_arr(4);
    int write_result = 1;
    for (auto const& pair : pos_map) {
        RCLCPP_INFO(this->get_logger(), "checking pos_map: k: 0x%x v: %f",
                    pair.first, pair.second);
    }

    for (auto const pair : pos_map) {
        auto id = pair.first;
        auto pos = pair.second;
        RCLCPP_INFO(this->get_logger(), "top of setServoPos loop: 0x%x, %f", id,
                    pos);

        if (servoMap_.find(id) != servoMap_.end()) {
            servoMap_[id].goalPos_ = pos;

            pos_arr = degToPos(pos);
            
            RCLCPP_INFO(this->get_logger(),
                        "inside setServoPos loop: id 0x%x pos %f, pos_arr %x%x%x%x",
                        id, pos, pos_arr[3], pos_arr[2], pos_arr[1], pos_arr[0]);
            map.insert({id, pos_arr});
            // RCLCPP_INFO(this->get_logger(),
            //             "Trying to set servo ID: %x to position %f", id,
            //             pos);
            // RCLCPP_INFO(this->get_logger(), "Pos in hex is 0x%x%x%x%x",
            //             pos_arr[3], pos_arr[2], pos_arr[1], pos_arr[0]);
        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "Servo ID: 0x%x is not in the list of instantiated servos", id);
        }
    }
    for (auto const& pair : map) {
        RCLCPP_INFO(this->get_logger(), "setServoPos: k: 0x%x v: 0x%x%x%x%x",
                    pair.first, pair.second[3], pair.second[2], pair.second[1],
                    pair.second[0]);
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

DynamixelDriver::~DynamixelDriver() {
    toggle_torque(allIds_, false);
    portHandler_->closePort();
}
