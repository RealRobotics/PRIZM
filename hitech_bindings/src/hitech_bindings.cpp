#include "hitech_bindings/hitech_bindings.hpp"

uint16_t positionToHex(int pos) { return float(pos * 16384) / 360; }

HitechBindings::HitechBindings(std::string& nodeName,
                               rclcpp::NodeOptions& options)
    : Node(nodeName, options) {
    initParameters();
    initContainers();
    initSubscribers();
    initPublishers();
    initServices();
}

void HitechBindings::initParameters() {
    // number of instances parameters
    auto num_motors_map =
        std::map<std::string, int>{{"num_servos", 1}, {"num_motors", 1}};

    this->declare_parameters(paramNamespace_, num_motors_map);
    numServos_ = this->get_parameter(paramNamespace_ + ".num_servos").as_int();
    numMotors_ = this->get_parameter(paramNamespace_ + ".num_motors").as_int();

    // limits parameters
    auto servo_limits_map = std::map<std::string, std::vector<int64_t>>{
        {"servo_mins", std::vector<int64_t>()},
        {"servo_maxs", std::vector<int64_t>()},
        {"servo_zeros", std::vector<int64_t>()},
    };

    auto motor_parameter_map = std::map<std::string, std::vector<int64_t>>{
        {"speed_mins", std::vector<int64_t>()},
        {"speed_maxs", std::vector<int64_t>()},
    };

    this->declare_parameters(paramNamespace_, servo_limits_map);
    this->declare_parameters(paramNamespace_, motor_parameter_map);

    servoMins_ =
        this->get_parameter(paramNamespace_ + ".servo_mins").as_integer_array();
    servoMaxs_ =
        this->get_parameter(paramNamespace_ + ".servo_maxs").as_integer_array();
    servoZeros_ = this->get_parameter(paramNamespace_ + ".servo_zeros")
                      .as_integer_array();

    speedMins_ =
        this->get_parameter(paramNamespace_ + ".speed_mins").as_integer_array();
    speedMaxs_ =
        this->get_parameter(paramNamespace_ + ".speed_maxs").as_integer_array();

    // instances IDs parameters
    auto ids_parameter_map = std::map<std::string, std::vector<int64_t>>{
        {"servo_ids", std::vector<int64_t>()},
        {"motor_ids", std::vector<int64_t>()},
    };
    this->declare_parameters(paramNamespace_, ids_parameter_map);
    servoIds_ =
        this->get_parameter(paramNamespace_ + ".servo_ids").as_integer_array();
    motorIds_ =
        this->get_parameter(paramNamespace_ + ".motor_ids").as_integer_array();
}

void HitechBindings::initContainers() {
    auto op_mode_map = std::map<uint8_t, uint8_t>();

    try {
        if (numServos_ != (int)servoIds_.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Mismatch in number of servos: %d and number of servo ids: %d",
                numServos_, (int)servoIds_.size());
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
            hitechData servo_default;
            servo_default.run_mode = RunMode::MULTI_TURN;
            for (int i = 0; i < numServos_; i++) {
                servo_default.servoMin = positionToHex(servoMins_.at(i));
                servo_default.servoMax = positionToHex(servoMaxs_.at(i));
                servo_default.servoZero = positionToHex(servoZeros_.at(i));
                servoMap_.insert({{servoIds_[i], servo_default}});
                op_mode_map.insert({servoIds_[i], servo_default.run_mode});
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    try {
        if (numMotors_ != (int)motorIds_.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Mismatch in number of motors: %d and number of motor ids: %d",
                numMotors_, (int)motorIds_.size());
            throw std::runtime_error("MotorNumMismatch");
        } else {
            hitechData motor_default;
            motor_default.run_mode = RunMode::SPEED;
            for (int i = 0; i < numMotors_; i++) {
                motorMap_.insert({motorIds_[i], motor_default});
                op_mode_map.insert({motorIds_[i], motor_default.run_mode});
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
    // summary of instanatiation
    // RCLCPP_INFO(this->get_logger(),
    //             "I have instantiated %d servos and %d motors.", numServos_,
    //             numMotors_);
}

void HitechBindings::initSubscribers() {
    serialSub_ = this->create_subscription<uint8Arr>(
        "serial_read", 10,
        std::bind(&HitechBindings::serialReadCB_, this, std::placeholders::_1));

    hitechPosSub_ = this->create_subscription<HitechPosition>(
        "/position_input", 10,
        std::bind(&HitechBindings::posSubCB_, this, std::placeholders::_1));
}

void HitechBindings::initServices() {
    setAngleSrv_ = this->create_service<SetHitechAngle>(
        "/set_angle", std::bind(&HitechBindings::setAngleCB_, this,
                                std::placeholders::_1, std::placeholders::_2));

    setStreamSrv_ = this->create_service<SetStreamRegisters>(
        "/set_stream_params",
        std::bind(&HitechBindings::setStreamCB_, this, std::placeholders::_1,
                  std::placeholders::_2));
    setPositionSrv_ = this->create_service<SetHitechPosition>(
        "/set_position",
        std::bind(&HitechBindings::setPositionCB_, this, std::placeholders::_1,
                  std::placeholders::_2));
}

void HitechBindings::initPublishers() {
    anglePub_ = this->create_publisher<HitechAngle>("HitechData", 10);
    positionPub_ = this->create_publisher<HitechPosition>("HitechPosition", 10);
    serialPub_ = this->create_publisher<uint8Arr>("/serial_write", 10);
}

void HitechBindings::setServoPos(int id, int target_angle) {
    uint16_t targetHex = positionToHex(target_angle);
    // 1. check that the ID is valid
    if (servoMap_.find(id) != servoMap_.end()) {
        // 2. check that inputs is in bounds
        if (target_angle < servoMap_[id].servoMax &&
            target_angle > servoMap_[id].servoMin) {
            // 3. check that the target is not the same as the current
            if (servoMap_[id].currAngle != targetHex) {
                // 4. update structs
                servoMap_[id].goalAngle = targetHex;
                singleWriteBuilder(WriteTasks::NEW_POSITION, id, targetHex);
                servoMap_[id].currAngle = targetHex;
            }
        }

    } else {  // we ain't got it
    }
}

std::vector<uint8_t> HitechBindings::singleWriteBuilder(WriteTasks task, int id,
                                                        int target) {
    std::vector<uint8_t> output(5);
    output.at(4) = target >> 8;      // low byte
    output.at(3) = (target & 0xFF);  // high byte
    output.at(2) = task;
    output.at(1) = id;
    output.at(0) = 'w';

    // @todo publish this downstream
    return output;
}

std::vector<uint8_t> HitechBindings::doubleWriteBuilder(WriteTasks task1,
                                                        WriteTasks task2,
                                                        int id, int target1,
                                                        int target2) {
    std::vector<uint8_t> output(8);
    output.at(7) = target2 >> 8;      // low byte
    output.at(6) = (target2 & 0xFF);  // high byte
    output.at(5) = task2;
    output.at(4) = target1 >> 8;      // low byte
    output.at(3) = (target1 & 0xFF);  // high byte
    output.at(2) = task1;
    output.at(1) = id;
    output.at(0) = 'W';

    return output;
}

void HitechBindings::serialReadCB_(const uint8Arr& msg) {
    size_t data_len = msg.data.size();
    // fork 1: len 5 or 8
    switch (data_len) {
        case 5:
            // single read
            singleReadHandler(msg);
            break;
        case 8:
            // double read
            doubleReadHandler(msg);
            break;

        default:
            RCLCPP_ERROR(this->get_logger(),
                         "Message received was len %ld which is neither 5 or 8 "
                         "byte long. Check "
                         "if the data needs tidying or ignore if data was bad.",
                         data_len);
            break;
    }
    publishServoInfo();
}

void HitechBindings::singleReadHandler(const uint8Arr& msg) {
    int servo_id = msg.data.at(1);
    int reg = msg.data.at(2);
    int data = msg.data.at(4) * 256 + msg.data.at(3);
    readHandler(servo_id, reg, data);
}

void HitechBindings::doubleReadHandler(const uint8Arr& msg) {
    int servo_id = msg.data.at(1);
    int reg1 = msg.data.at(2);
    int data1 = msg.data.at(4) * 256 + msg.data.at(3);
    int reg2 = msg.data.at(5);
    int data2 = msg.data.at(7) * 256 + msg.data.at(6);
    readHandler(servo_id, reg1, data1);
    readHandler(servo_id, reg2, data2);
}

void HitechBindings::readHandler(uint8_t servo_id, uint8_t addr,
                                 uint16_t data) {
    if (servoMap_.find(servo_id) != servoMap_.end()) {
        switch (addr) {
            case ReadTasks::POSITION:
                servoMap_[servo_id].currAngle = data;
                break;
            case ReadTasks::TURN:
                servoMap_[servo_id].currTurn = data;
                break;
            case ReadTasks::VOLTAGE:
                servoMap_[servo_id].voltage = data;
                break;
            case ReadTasks::TORQUE:
                servoMap_[servo_id].torque = data;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(),
                             "Read register 0x%X is not in implemented list. "
                             "Check the ReadTasks enum.",
                             addr);
                break;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Servo id read was not in the list of instantiated "
                     "servos. Printing the values only");
    }
}

void HitechBindings::publishServoInfo() {
    HitechAngle data;
    HitechPosition position;

    data.motor_id.resize(numServos_);
    data.motor_angle.resize(numServos_);
    data.motor_turn.resize(numServos_);
    data.size = numServos_;

    position.motor_id.resize(numServos_);
    position.motor_pos.resize(numServos_);
    position.size = numServos_;

    int counter = 0;
    for (auto const& [key, val] : servoMap_) {
        data.motor_id.at(counter) = key;
        position.motor_id.at(counter) = key;
        data.motor_angle.at(counter) = val.currAngle;
        data.motor_turn.at(counter) = val.currTurn;
        // position.motor_pos.at(counter) = val.currAngle
        counter++;
    }

    anglePub_->publish(data);
}

void HitechBindings::publishSerialCommand(std::vector<uint8_t> message) {
    uint8Arr outgoing;
    outgoing.data = message;

    serialPub_->publish(outgoing);
}

void HitechBindings::setAngleCB_(const SetHitechAngle::Request::SharedPtr req,
                                 SetHitechAngle::Response::SharedPtr res) {
    // get the data out of the request
    int num_incoming_angles = req->hitech_angle.size;
    res->positions.resize(num_incoming_angles);

    for (int i = 0; i < num_incoming_angles; i++) {
        int servo_id = req->hitech_angle.motor_id[i];
        int target_angle = req->hitech_angle.motor_angle[i];
        int target_turn = req->hitech_angle.motor_turn[i];
        auto instruction =
            doubleWriteBuilder(WriteTasks::NEW_POSITION, WriteTasks::NEW_TURN,
                               servo_id, target_angle, target_turn);
        // res->angles
        res->positions[i] = target_angle / 4096 * 90 + target_turn * 360;
        publishSerialCommand(instruction);
    }
    res->success = true;
}

void HitechBindings::setPositionCB_(
    const SetHitechPosition::Request::SharedPtr req,
    SetHitechPosition::Response::SharedPtr res) {
    int num_incoming_positions = req->hitech_position.size;
    res->positions.resize(num_incoming_positions);
    for (int i = 0; i < num_incoming_positions; i++) {
        int servo_id = req->hitech_position.motor_id[i];
        int position = req->hitech_position.motor_pos[i];
        position = position * 90 / 4096;
        int turn = position / 360;
        int angle = position % 360;
        auto instruction =
            doubleWriteBuilder(WriteTasks::NEW_POSITION, WriteTasks::NEW_TURN,
                               servo_id, angle, turn);
        publishSerialCommand(instruction);
        res->positions[i] = position;
    }
    res->success = true;
}

void HitechBindings::updateServoPos() {
    for (auto const& [key, val] : servoMap_) {
        servoMap_[key].currPos = servoMap_[key].currAngle * (4096 / 90) +
                                 servoMap_[key].currTurn * 360;
    }
}

void HitechBindings::setStreamCB_(
    const SetStreamRegisters::Request::SharedPtr req,
    SetStreamRegisters::Response::SharedPtr res) {
    // if(req->toggle != )
    for (auto& setting : req->stream_settings) {
        // 1. check if the id is in the cached list
        int incoming_id = setting.servo_id;
        if (servoMap_.find(incoming_id) != servoMap_.end()) {
            // 2. check the streaming time
            if (servoMap_.at(incoming_id).stream_time != setting.stream_time) {
                servoMap_.at(incoming_id).stream_time = setting.stream_time;
                setStreamTime(incoming_id, setting.stream_time);
            }
            // 3. check the toggle
            if (servoMap_.at(incoming_id).stream_time != setting.toggle) {
                servoMap_.at(incoming_id).stream_toggle = setting.toggle;
                toggleStreamMode(incoming_id, setting.toggle);
            }
            // 4. check the parameters
            /**
             * @note I have spent a while thinking about how to make this
             * clever, checking cached values and all that jazz. Then I realsied
             * we will call this once in a blue moon and I doubt the extra
             * computation will actually hurt us all that much.
             * In short, apologies if it is a bit long, but at least it'll
             * definitely be ligible.
             *
             */
            // 4.0 param 0
            switch (setting.addr_0.size()) {
                case 1:
                    servoMap_.at(incoming_id).stream_00 = setting.addr_0.at(0);
                    setStreamReg(incoming_id, WriteTasks::REG_STREAM_0,
                                 setting.addr_0.at(0), 0x00);
                    break;
                case 2:
                    servoMap_.at(incoming_id).stream_00 = setting.addr_0.at(0);
                    servoMap_.at(incoming_id).stream_01 = setting.addr_0.at(1);
                    setStreamReg(incoming_id, WriteTasks::REG_STREAM_0,
                                 setting.addr_0.at(0), setting.addr_0.at(1));
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(),
                                 "Either more than 2 or less than 1 stream "
                                 "registers were sent to addr_0. Ignoring.");
                    break;
            }
            // 4.1 param 1
            switch (setting.addr_1.size()) {
                case 1:
                    servoMap_.at(incoming_id).stream_10 = setting.addr_1.at(0);
                    setStreamReg(incoming_id, WriteTasks::REG_STREAM_1,
                                 setting.addr_1.at(0), 0x00);
                    break;
                case 2:
                    servoMap_.at(incoming_id).stream_10 = setting.addr_1.at(0);
                    servoMap_.at(incoming_id).stream_11 = setting.addr_1.at(1);
                    setStreamReg(incoming_id, WriteTasks::REG_STREAM_1,
                                 setting.addr_1.at(0), setting.addr_1.at(1));
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(),
                                 "Either more than 2 or less than 1 stream "
                                 "registers were sent to addr_1. Ignoring.");
                    break;
            }
            // 4.2 param 2
            switch (setting.addr_2.size()) {
                case 1:
                    servoMap_.at(incoming_id).stream_20 = setting.addr_2.at(0);
                    setStreamReg(incoming_id, WriteTasks::REG_STREAM_2,
                                 setting.addr_2.at(0), 0x00);
                    break;
                case 2:
                    servoMap_.at(incoming_id).stream_20 = setting.addr_2.at(0);
                    servoMap_.at(incoming_id).stream_21 = setting.addr_2.at(1);
                    setStreamReg(incoming_id, WriteTasks::REG_STREAM_2,
                                 setting.addr_2.at(0), setting.addr_2.at(1));
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(),
                                 "Either more than 2 or less than 1 stream "
                                 "registers were sent to addr_2. Ignoring.");
                    break;
            }
            // 4.3 param 3
            switch (setting.addr_3.size()) {
                case 1:
                    servoMap_.at(incoming_id).stream_30 = setting.addr_3.at(0);
                    setStreamReg(incoming_id, WriteTasks::REG_STREAM_3,
                                 setting.addr_3.at(0), 0x00);
                    break;
                case 2:
                    servoMap_.at(incoming_id).stream_30 = setting.addr_3.at(0);
                    servoMap_.at(incoming_id).stream_31 = setting.addr_3.at(1);
                    setStreamReg(incoming_id, WriteTasks::REG_STREAM_3,
                                 setting.addr_3.at(0), setting.addr_3.at(1));
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(),
                                 "Either more than 2 or less than 1 stream "
                                 "registers were sent to addr_3. Ignoring.");
                    break;
            }
        }
    }
    res->success = true;
}

void HitechBindings::setStreamTime(uint8_t servo_id, int stream_time) {
    singleWriteBuilder(WriteTasks::STREAM_TIME, servo_id, stream_time);
}

void HitechBindings::toggleStreamMode(uint8_t servo_id, bool toggle) {
    singleWriteBuilder(WriteTasks::STREAM_TIME, servo_id, toggle);
}

void HitechBindings::setStreamReg(uint8_t servo_id, WriteTasks stream_addr,
                                  uint8_t reg1, uint8_t reg2) {
    uint16_t stream_reg_combined = reg1 | (reg2 << 8);
    singleWriteBuilder(stream_addr, servo_id, stream_reg_combined);
}

WriteTasks HitechBindings::WriteTaskfromUint(uint8_t val) {
    switch (val) {
        case WriteTasks::NEW_POSITION:
            return WriteTasks::NEW_POSITION;
        case WriteTasks::NEW_TURN:
            return WriteTasks::NEW_TURN;
        case WriteTasks::SERVO_MAX:
            return WriteTasks::SERVO_MAX;
        case WriteTasks::SERVO_MIN:
            return WriteTasks::SERVO_MIN;
        case WriteTasks::SERVO_ZERO:
            return WriteTasks::SERVO_ZERO;
        case WriteTasks::NEW_SPEED:
            return WriteTasks::NEW_SPEED;
        case WriteTasks::STREAM_MODE:
            return WriteTasks::STREAM_MODE;
        case WriteTasks::STREAM_TIME:
            return WriteTasks::STREAM_TIME;
        case WriteTasks::REG_STREAM_0:
            return WriteTasks::REG_STREAM_0;
        case WriteTasks::REG_STREAM_1:
            return WriteTasks::REG_STREAM_1;
        case WriteTasks::REG_STREAM_2:
            return WriteTasks::REG_STREAM_2;
        case WriteTasks::REG_STREAM_3:
            return WriteTasks::REG_STREAM_3;

        default:
            return WriteTasks::ERROR;
    }
}

void HitechBindings::posSubCB_(const HitechPosition& msg) {
    int num_incoming_pos = msg.size;
    for (int i = 0; i < num_incoming_pos; i++) {
        int req_id = msg.motor_id[i];
        if (servoMap_.find(req_id) != servoMap_.end()) {
            int angle = msg.motor_pos.at(i) % 16384;
            int turn = msg.motor_pos.at(i) / 16384;
            servoMap_.at(req_id).goalAngle = angle;
            servoMap_.at(req_id).goalTurn = turn;
            publishSerialCommand(doubleWriteBuilder(WriteTasks::NEW_POSITION,
                                                    WriteTasks::NEW_TURN,
                                                    req_id, angle, turn));
            servoMap_.at(req_id).currAngle = angle;
            servoMap_.at(req_id).currTurn = turn;
        }
    }
}
