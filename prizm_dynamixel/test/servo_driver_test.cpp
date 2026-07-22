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
    ASSERT_EQ(node_->servoList_[2].motorId_, 0x02);
    ASSERT_EQ(node_->motorList_[1].motorId_, 0x09);
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

    ASSERT_EQ(node_->servoList_[0].goalPos_, p1.position);
    ASSERT_EQ(node_->servoList_[1].goalPos_, p2.position);
    ASSERT_EQ(node_->servoList_[2].goalPos_, p3.position);
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

    ASSERT_EQ(node_->motorList_[0].goalVel_, p1.velocity);
    ASSERT_EQ(node_->motorList_[1].goalVel_, p2.velocity);
    ASSERT_EQ(node_->motorList_[2].goalVel_, p3.velocity);
}



int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    RclCppFixture g_rclcppfixture(argc, argv);
    return RUN_ALL_TESTS();
}