#include "prizm_dynamixel/single_motor.hpp"

#define DEVICE_PORT "/dev/ttyUSB0"
#define BAUD_RATE 57600
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PROFILE_ACCEL 108
#define ADDR_PROFILE_VELOCITY 112

SingleMotor::SingleMotor(dynamixel::PortHandler* portHandler,
                         dynamixel::PacketHandler* packetHandler)
    : Node("single_motor") {
    port_handler_ = portHandler;
    packet_handler_ = packetHandler;

    RCLCPP_INFO(this->get_logger(), "Running Single Motor Node");
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth))
                                .reliable()
                                .durability_volatile();

    velocity_sub_ = this->create_subscription<DynaVel>(
        "dyna_set_vel", QOS_RKL10V,
        std::bind(&SingleMotor::set_velocity_callback, this, _1));

    get_velocity_srv_ = this->create_service<DynaGetVel>(
        "dyna_get_vel",
        std::bind(&SingleMotor::get_velocity_service, this, _1, _2));
}

void SingleMotor::set_velocity_callback(const DynaVel::SharedPtr msg) {
    uint8_t dxl_error = 0x00;

    uint32_t goal_vel = (unsigned int)msg->velocity;
    uint32_t goal_accl = (unsigned int)msg->acceleration;

    if (msg->acceleration != current_acc) {
        auto dxl_comm_result = packet_handler_->write4ByteTxRx(
            port_handler_, (uint8_t)msg->id, ADDR_PROFILE_ACCEL, goal_accl,
            &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "%s",
                        packet_handler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            RCLCPP_INFO(this->get_logger(), "%s",
                        packet_handler_->getRxPacketError(dxl_error));
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Set [ID: %d] [Profile acceleration: %d]", msg->id,
                        msg->acceleration);
        }
    }

    // set the velocity goal
    auto dxl_comm_result = packet_handler_->write4ByteTxRx(
        port_handler_, (uint8_t)msg->id, ADDR_GOAL_VELOCITY, goal_vel,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s",
                    packet_handler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s",
                    packet_handler_->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal velocity: %d]",
                    msg->id, msg->velocity);
    }
}

void SingleMotor::get_velocity_service(
    std::shared_ptr<DynaGetVel::Request> request,
    std::shared_ptr<DynaGetVel::Response> response) {
    uint8_t dxl_error = 0x00;

    auto dxl_comm_result = packet_handler_->read4ByteTxRx(
        port_handler_, (uint8_t)request->id, ADDR_PRESENT_VELOCITY,
        reinterpret_cast<uint32_t*>(&current_vel), &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("single_motor"),
                     "Failed to get velocity.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("single_motor"),
                    "Succeeded to get velocity.");
    }

    auto dxl_comm_result = packet_handler_->read4ByteTxRx(
        port_handler_, (uint8_t)request->id, ADDR_PROFILE_ACCEL,
        reinterpret_cast<uint32_t*>(&current_acc, &dxl_error));

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("single_motor"),
                     "Failed to get acceleration.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("single_motor"),
                    "Succeeded to get acceleration.");
    }

    RCLCPP_INFO(this->get_logger(),
                "Get [ID: %d] [Present velocity: %d] [Present accelration: %d]",
                request->id, current_vel, current_acc);

    response->velocity = current_vel;
    response->acceleration = current_acc;
}

SingleMotor::~SingleMotor() {}

void setupDynamixel(uint8_t dxl_id, dynamixel::PortHandler* portHandler,
                    dynamixel::PacketHandler* packetHandler,
                    uint8_t dxl_error = 0) {
    // Use Velocity Control Mode
    auto dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, dxl_id, ADDR_OPERATING_MODE, 1, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                     "Failed to set velocity Control Mode.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                    "Succeeded to set velocity Control Mode.");
    }

    // Set acceleration mode
    auto dxl_comm_result = packetHandler->write4ByteTxRx(
        portHandler, dxl_id, ADDR_PROFILE_ACCEL, 200, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                     "Failed to set the profile acceleration.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                    "Succeeded to set the profile acceleration.");
    }

    // Enable Torque of DYNAMIXEL
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                     "Failed to enable torque.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                    "Succeeded to enable torque.");
    }
}

int main(int argc, char* argv[]) {
    try {
        rclcpp::init(argc, argv);

    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    auto portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_PORT);
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler();
    uint8_t dxl_error = 0x00;
    bool dx1_comm_result = portHandler->openPort();
    if (!dx1_comm_result) {
        RCLCPP_ERROR(rclcpp::get_logger("single_motor"),
                     "Could not open the port");
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("single_motor"),
                    "Port opened successfully");
    }
    dx1_comm_result = portHandler->setBaudRate(BAUD_RATE);
    if (!dx1_comm_result) {
        RCLCPP_ERROR(rclcpp::get_logger("single_motor"),
                     "Could not set baud rate");
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("single_motor"),
                    "Baud rate set successfully");
    }

    setupDynamixel(BROADCAST_ID, portHandler, packetHandler);
    auto single_motor =
        std::make_shared<SingleMotor>(portHandler, packetHandler);

    rclcpp::spin(single_motor);
    rclcpp::shutdown();

    packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE,
                                  0, &dxl_error);

    return 1;
}