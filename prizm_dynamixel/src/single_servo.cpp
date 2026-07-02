#include "prizm_dynamixel/single_servo.hpp"

#define DEVICE_PORT "/dev/ttyUSB0"
#define BAUD_RATE 57600
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

SingleServo::SingleServo(dynamixel::PortHandler* portHandler,
                         dynamixel::PacketHandler* packetHandler)
    : Node("singl_servo") {
    port_handler_ = portHandler;
    packet_handler_ = packetHandler;

    RCLCPP_INFO(this->get_logger(), "Running Single Servo Node");
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth))
                                .reliable()
                                .durability_volatile();

    position_sub_ = this->create_subscription<DynaPos>(
        "dyna_set_pos", QOS_RKL10V,
        std::bind(&SingleServo::set_position_callback, this, _1));

    get_position_srv_ = this->create_service<DynaGetPos>(
        "dyna_get_pos",
        std::bind(&SingleServo::get_position_service, this, _1, _2));
}

void SingleServo::set_position_callback(const DynaPos::SharedPtr msg) {
    uint8_t dxl_error = 0x00;

    uint32_t goal_pos = (unsigned int)msg->position;

    auto dxl_comm_result = packet_handler_->write4ByteTxRx(
        port_handler_, (uint8_t)msg->id, ADDR_GOAL_POSITION, goal_pos,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s",
                    packet_handler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s",
                    packet_handler_->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]",
                    msg->id, msg->position);
    }
}

void SingleServo::get_position_service(
    std::shared_ptr<DynaGetPos::Request> request,
    std::shared_ptr<DynaGetPos::Response> response) {
    uint8_t dxl_error = 0x00;
    auto dxl_comm_result = packet_handler_->read4ByteTxRx(
        port_handler_, (uint8_t)request->id, ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t*>(&current_pos), &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("single_servo"),
                     "Failed to get position.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("single_servo"),
                    "Succeeded to get position.");
    }

    RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present Position: %d]",
                request->id, current_pos);

    response->position = current_pos;
}

SingleServo::~SingleServo() {}

void setupDynamixel(uint8_t dxl_id, dynamixel::PortHandler* portHandler,
                    dynamixel::PacketHandler* packetHandler,
                    uint8_t dxl_error = 0) {
    // Use Position Control Mode
    auto dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, dxl_id, ADDR_OPERATING_MODE, 3, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"),
                     "Failed to set Position Control Mode.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"),
                    "Succeeded to set Position Control Mode.");
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
    try
    {
        rclcpp::init(argc, argv);

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    auto portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_PORT);
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler();
    uint8_t dxl_error = 0x00;
    bool dx1_comm_result = portHandler->openPort();
    if (!dx1_comm_result) {
        RCLCPP_ERROR(rclcpp::get_logger("single_servo"),
                     "Could not open the port");
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("single_servo"),
                    "Port opened successfully");
    }
    dx1_comm_result = portHandler->setBaudRate(BAUD_RATE);
    if (!dx1_comm_result) {
        RCLCPP_ERROR(rclcpp::get_logger("single_servo"),
                     "Could not set baud rate");
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("single_servo"),
                    "Baud rate set successfully");
    }

    setupDynamixel(BROADCAST_ID, portHandler, packetHandler);
    auto single_servo =
        std::make_shared<SingleServo>(portHandler, packetHandler);

    
    
    rclcpp::spin(single_servo);
    rclcpp::shutdown();

    packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE,
                                  0, &dxl_error);

    return 1;
}