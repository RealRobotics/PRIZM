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
    ASSERT_EQ(node_->motorMap_[0x09].torqueEnable_, true);
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

TEST_F(TestMotorDriver, SetSpeed){
    uint8_t id1 = 0x06;
    uint8_t id2 = 0x09;
    std::map<uint8_t, float> vel_input = {
        {0x06, 20.14},
        {0x09, -11.44}
    };
    node_->setGoalVelocity(vel_input);
    auto vel_1 = node_->motorMap_[id1].goalVel_;
    auto vel_2 = node_->motorMap_[id2].goalVel_;
    ASSERT_NEAR(vel_1, 20.14, 1e-1);
    ASSERT_NEAR(vel_2, -11.44, 1e-1);

    // little extra, check the conversions
    uint8_t hex[4];
    node_->rpmToHex(20.14, hex);
    uint8_t ans[4];
    ans[3] = 0x00;
    ans[2] = 0x00;
    ans[1] = 0x00;
    ans[0] = 0x58;

    EXPECT_EQ(ans[0], hex[0]);
    EXPECT_EQ(ans[1], hex[1]);
    EXPECT_EQ(ans[2], hex[2]);
    EXPECT_EQ(ans[3], hex[3]);

    node_->rpmToHex(-11.44, hex);
    ans[3] = 0xFF;
    ans[2] = 0xFF;
    ans[1] = 0xFF;
    ans[0] = 0xCE;
    EXPECT_EQ(ans[0], hex[0]);
    EXPECT_EQ(ans[1], hex[1]);
    EXPECT_EQ(ans[2], hex[2]);
    EXPECT_EQ(ans[3], hex[3]);
}

TEST_F(TestMotorDriver, SetServoPos){
    uint8_t id1 = 0x00;
    uint8_t id2 = 0x01;
    uint8_t id3 = 0x02;
    float goal1 = -43.95, goal2 = 105.47, goal3 = 439.45;
    std::map<uint8_t, float> pos_input = {
        {id1, goal1},
        {id2, goal2},
        {id3, goal3}
    };
    node_->setServoPos(pos_input);
    auto pos1 = node_->servoMap_[id1].goalPos_;
    auto pos2 = node_->servoMap_[id2].goalPos_;
    auto pos3 = node_->servoMap_[id3].goalPos_;
    ASSERT_NEAR(pos1, goal1, 1e-1);
    ASSERT_NEAR(pos2, goal2, 1e-1);
    ASSERT_NEAR(pos3, goal3, 1e-1);

    // little extra, check the conversions
    uint8_t hex[4];
    node_->degToPos(goal1, hex);
    uint8_t ans[4];
    ans[3] = 0xFF;
    ans[2] = 0xFF;
    ans[1] = 0xFE;
    ans[0] = 0x0C;

    EXPECT_EQ(ans[0], hex[0]);
    EXPECT_EQ(ans[1], hex[1]);
    EXPECT_EQ(ans[2], hex[2]);
    EXPECT_EQ(ans[3], hex[3]);

    node_->degToPos(goal2, hex);
    ans[3] = 0x00;
    ans[2] = 0x00;
    ans[1] = 0x04;
    ans[0] = 0xB0;
    EXPECT_EQ(ans[0], hex[0]);
    EXPECT_EQ(ans[1], hex[1]);
    EXPECT_EQ(ans[2], hex[2]);
    EXPECT_EQ(ans[3], hex[3]);

    node_->degToPos(goal3, hex);
    ans[3] = 0x00;
    ans[2] = 0x00;
    ans[1] = 0x13;
    ans[0] = 0x88;
    EXPECT_EQ(ans[0], hex[0]);
    EXPECT_EQ(ans[1], hex[1]);
    EXPECT_EQ(ans[2], hex[2]);
    EXPECT_EQ(ans[3], hex[3]);

    node_->degToPos(-439.45, hex);
    ans[3] = 0xFF;
    ans[2] = 0xFF;
    ans[1] = 0xEC;
    ans[0] = 0x78;
    EXPECT_EQ(ans[0], hex[0]);
    EXPECT_EQ(ans[1], hex[1]);
    EXPECT_EQ(ans[2], hex[2]);
    EXPECT_EQ(ans[3], hex[3]);
}


int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    RclCppFixture g_rclcppfixture(argc, argv);
    return RUN_ALL_TESTS();
}