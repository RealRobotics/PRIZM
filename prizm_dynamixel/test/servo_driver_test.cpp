#include <gtest/gtest.h>

#include <prizm_dynamixel/dynamixelDriver.hpp>

// messages for callback testing
#include "prizm_interfaces/msg/dynamixel_pos.hpp"
#include "prizm_interfaces/msg/dynamixel_pos_arr.hpp"
#include "prizm_interfaces/msg/dynamixel_vel.hpp"
#include "prizm_interfaces/msg/dynamixel_vel_arr.hpp"

// Take in argc and argv if you want to get_parameters for node
class RclCppFixture {
   public:
    RclCppFixture(int a, char** v) { rclcpp::init(a, v); }
    RclCppFixture() { rclcpp::init(0, nullptr); }
    ~RclCppFixture() { rclcpp::shutdown(); }
};

class TestMotorDriver : public ::testing::Test {
   protected:
    void SetUp();
    void spin();
    void TearDown();

    std::shared_ptr<DynamixelDriver> node_;

   private:
    std::string nodeName = "motor_driver_test";
    std::string paramNamespace_ = "motor_driver";

   public:
    void PublishWrongNumberPos();
};

void TestMotorDriver::SetUp() {
    // rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    std::vector<uint8_t> servoIds = {0x00, 0x01, 0x02};
    std::vector<uint8_t> motorIds = {0x06, 0x09, 0x04};

    node_ = std::make_shared<DynamixelDriver>(nodeName, options);
}

void TestMotorDriver::TearDown() {
    node_.reset();
    // rclcpp::shutdown();
}

void TestMotorDriver::PublishWrongNumberPos() {
    auto posArrayPub = node_->create_publisher<DynaPosArr>("servo_pos", 10);
    DynaPos p1, p2;
    p1.id = 0x00;
    p2.id = 0x01;

    p1.position = 180;
    p2.position = 30;

    DynaPosArr pos_arr;
    pos_arr.pos_array = {p1, p2};
    posArrayPub->publish(pos_arr);

    rclcpp::spin_some(node_);
}

TEST_F(TestMotorDriver, AddressSetting) {
    ASSERT_EQ("/dev/ttyUSB2", node_->address);
}

TEST_F(TestMotorDriver, InitMotors) {
    auto servo_it = node_->servoMap_.find(0x02);
    auto motor_it = node_->motorMap_.find(0x09);
    ASSERT_EQ(servo_it != node_->servoMap_.end(), true);
    ASSERT_EQ(motor_it != node_->motorMap_.end(), true);
    // ASSERT_EQ(node_->servoMap_[0x02], 0x02);
    // ASSERT_EQ(node_->motorMap_[0x09], 0x09);
}

// TEST_F(TestMotorDriver, InitServoMismatch) { FAIL(); }
// TEST_F(TestMotorDriver, InitMotorsMismatch) { FAIL(); }

TEST_F(TestMotorDriver, ServoPosCallback) {
    auto posArrayPub = node_->create_publisher<DynaPosArr>("servo_pos", 10);
    DynaPos p1, p2, p3;
    p1.id = 0x00;
    p2.id = 0x01;
    p3.id = 0x02;

    p1.position = 180;
    p2.position = 30;
    p3.position = 45;

    DynaPosArr pos_arr;
    pos_arr.pos_array = {p1, p2, p3};
    posArrayPub->publish(pos_arr);

    rclcpp::spin_some(node_);

    ASSERT_EQ(node_->servoMap_[p1.id].goalPos_, p1.position);
    ASSERT_EQ(node_->servoMap_[p2.id].goalPos_, p2.position);
    ASSERT_EQ(node_->servoMap_[p3.id].goalPos_, p3.position);
}

TEST_F(TestMotorDriver, MotorVelCallback) {
    auto velArrayPub = node_->create_publisher<DynaVelArr>("motor_vel", 10);
    DynaVel p1, p2, p3;
    p1.id = 0x06;
    p2.id = 0x09;
    p3.id = 0x04;

    p1.velocity = 180;
    p2.velocity = 30;
    p3.velocity = 45;

    DynaVelArr vel_arr;
    vel_arr.vel_array = {p1, p2, p3};
    velArrayPub->publish(vel_arr);

    rclcpp::spin_some(node_);

    ASSERT_EQ(node_->motorMap_[p1.id].goalVel_, p1.velocity);
    ASSERT_EQ(node_->motorMap_[p2.id].goalVel_, p2.velocity);
    ASSERT_EQ(node_->motorMap_[p3.id].goalVel_, p3.velocity);
}

TEST_F(TestMotorDriver, TorqueEnableTest) {
    std::vector<uint8_t> ids = {0x00, 0x02, 0x09};

    node_->enableTorque(ids);
    ASSERT_EQ(node_->servoMap_[0x00].torqueEnable_, true);
    ASSERT_EQ(node_->servoMap_[0x02].torqueEnable_, true);
    ASSERT_EQ(node_->servoMap_[0x01].torqueEnable_, false);
    ASSERT_EQ(node_->motorMap_[0x09].torqueEnable_, true);
    ASSERT_EQ(node_->motorMap_[0x04].torqueEnable_, false);
}

TEST_F(TestMotorDriver, ChangeOperatingModeTest) {
    std::map<uint8_t, uint8_t> op_mode_map = {
        {0x00, OperatingModes::EXTENDED_POSITION_CONTROL},
        {0x01, OperatingModes::POSITION_CONTROL},
        {0x04, OperatingModes::VELOCITY_CONTROL}};
    node_->changeOperatingMode(op_mode_map);

    ASSERT_EQ(node_->servoMap_[0x00].operatingMode_,
              OperatingModes::EXTENDED_POSITION_CONTROL);
    ASSERT_EQ(node_->servoMap_[0x01].operatingMode_,
              OperatingModes::POSITION_CONTROL);
    ASSERT_EQ(node_->motorMap_[0x04].operatingMode_,
              OperatingModes::VELOCITY_CONTROL);
}

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    RclCppFixture g_rclcppfixture(argc, argv);
    return RUN_ALL_TESTS();
}