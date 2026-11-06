#include "hitech_bindings/hitech_bindings.hpp"

uint16_t positionToHex(int pos) { return float(pos * 16384) / 360; }

HitechBindings::HitechBindings(std::string& nodeName,
                               rclcpp::NodeOptions& options)
    : Node(nodeName, options) {
    initParameters();
    initContainers();
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
    RCLCPP_INFO(this->get_logger(),
                "I have instantiated %d servos and %d motors.", numServos_,
                numMotors_);
}

void HitechBindings::setServoPos(int id, int target_angle) {
    uint16_t targetHex = positionToHex(target_angle);
    // 1. check that the ID is valid
    if (servoMap_.find(id) != servoMap_.end()) {
        // 2. check that inputs is in bounds
        if (target_angle < servoMap_[id].servoMax &&
            target_angle > servoMap_[id].servoMin) {
            // 3. check that the target is not the same as the current
            if (servoMap_[id].currPos != targetHex) {
                // 4. update structs
                servoMap_[id].goalAngle = targetHex;
                singleWriteBuilder(WriteTasks::NEW_POSITION, id, targetHex);
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

// int main(int argc, char* argv) { return 1; }