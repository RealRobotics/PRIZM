#include <rclcpp/rclcpp.hpp>
#include <prizm_interfaces/msg/dynamixel_pos.hpp>
#include <prizm_interfaces/msg/dynamixel_vel.hpp>
#include <prizm_interfaces/msg/dynamixel_pos_arr.hpp>
#include <prizm_interfaces/msg/dynamixel_vel_arr.hpp>

#include "prizm_dynamixel/dynamixelMotor.hpp"

// parameters len
#define GOAL_POS_LEN 4
#define GOAL_VEL_LEN 4
#define GOAL_ACCEL_LEN 4
#define TORQUE_ENABLE_LEN 1
#define CURR_TEMP_LEN 1
#define CURR_VOLT_LEN 2
#define OP_MODE_LEN 1
#define DRIVE_MODE_LEN 1

// parameters address
#define GOAL_POS_ADDR 116
#define GOAL_VEL_ADDR 104
#define GOAL_ACCEL_ADDR 108
#define TORQUE_ENABLE_ADDR 64
#define CURR_TEMP_ADDR 146
#define CURR_VOLT_ADDR 144
#define OP_MODE_ADDR 11
#define DRIVE_MODE_ADDR 10

enum {
    FWD_DEMDAND = 0x08,
    BWD_DEMAND = 0x09,
    FWD_TOGGLE = 0x00,
    BWD_TOGGLE = 0x01
};

typedef prizm_interfaces::msg::DynamixelPos DynaPos;
typedef prizm_interfaces::msg::DynamixelVel DynaVel;
typedef prizm_interfaces::msg::DynamixelPosArr DynaPosArr;
typedef prizm_interfaces::msg::DynamixelVelArr DynaVelArr;

class DynamixelDriver : public DynamixelMotor, public rclcpp::Node {
   public:
    // void InitServos();
    void InitMotors();
    void InitDynamixelComms();
    void InitGoalSubscribers();
    // uint8_t getMotorId() override;
    // uint32_t getOpMode() override;
    // uint32_t getTemp() override;
    // uint32_t getErrorStatus() override;
    // uint32_t getCurrentPos() override;
    // uint32_t getCurrentVel() override;
    // uint32_t getCurrentAccel() override;

    std::string address = "/dev/ttyUSBX";
    std::string paramNamespace_ = "motor_driver";
    int numServos_, servoMin_, servoMax_;
    int numMotors_, speedMin_, speedMax_;
    std::vector<dynamixelData> servoList_;
    std::vector<dynamixelData> motorList_;
    std::vector<int64_t> motorIdsINT_, servoIdsINT_;
    DynamixelDriver();
    DynamixelDriver(std::string& nodeName, rclcpp::NodeOptions& options);

    rclcpp::Subscription<DynaPosArr>::SharedPtr
        servoGoalPosSub_;
    rclcpp::Subscription<DynaVelArr>::SharedPtr
        motorGoalPosSub_;

    dynamixel::PortHandler* portHandler_;
    dynamixel::PacketHandler* packetHandler_;
    std::shared_ptr<dynamixel::GroupBulkRead> groupBulkRead_;
    std::shared_ptr<dynamixel::GroupBulkWrite> groupBulkWrite_;

   private:
    std::string nodeName_;

    void servoPosGoalCB_(const DynaPosArr &msg);
    void motorPosGoalCB_(const DynaVelArr &msg);

    void updateServoGoals();
    void updateMotorGoals();
};
