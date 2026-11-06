#include <gtest/gtest.h>

#include <hitech_bindings/hitech_bindings.hpp>
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

    std::shared_ptr<HitechBindings> node_;

   private:
    std::string nodeName = "motor_driver_test";
    std::string paramNamespace_ = "motor_driver";

   public:
    void PublishWrongNumberPos();
};

void TestMotorDriver::SetUp() {
    // rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    // std::vector<uint8_t> servoIds = {0x00, 0x01, 0x02};
    // std::vector<uint8_t> motorIds = {0x06, 0x09, 0x04};

    node_ = std::make_shared<HitechBindings>(nodeName, options);
}

void TestMotorDriver::TearDown() {
    node_.reset();
    // rclcpp::shutdown();
}

TEST_F(TestMotorDriver, InitContainers) { 
    auto servo_it = node_->servoMap_.find(0x10);
    auto motor_it = node_->motorMap_.find(0x06);
    ASSERT_EQ(servo_it != node_->servoMap_.end(), true);
    ASSERT_EQ(motor_it != node_->motorMap_.end(), true);
}

TEST_F(TestMotorDriver, SetSinglePosition) { 
    ASSERT_EQ(node_->servoMap_[0x10].currAngle, 0);
    node_->setServoPos(0x10, 200);
    ASSERT_EQ(node_->servoMap_[0x10].currAngle, int(200 * 16384 / 360) );
}

TEST_F(TestMotorDriver, BuildSingleInstruction){
    auto res = node_->singleWriteBuilder(WriteTasks::NEW_POSITION, 0x10, 200);
    ASSERT_EQ(res.at(0), 'w');
    ASSERT_EQ(res.at(1), 0x10);
    ASSERT_EQ(res.at(2), WriteTasks::NEW_POSITION);
    ASSERT_EQ(res.at(3), 0xC8);
    ASSERT_EQ(res.at(4), 0x00);

}

TEST_F(TestMotorDriver, ChangeOPMode) { FAIL(); }

TEST_F(TestMotorDriver, SetMultiplePosition) { FAIL(); }

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    RclCppFixture g_rclcppfixture(argc, argv);
    return RUN_ALL_TESTS();
}