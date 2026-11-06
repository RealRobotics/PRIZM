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
    std::vector<uint8_t> servoIds = {0x00, 0x01, 0x02};
    std::vector<uint8_t> motorIds = {0x06, 0x09, 0x04};

    node_ = std::make_shared<HitechBindings>(nodeName, options);
}

void TestMotorDriver::TearDown() {
    node_.reset();
    // rclcpp::shutdown();
}

TEST_F(TestMotorDriver, InitContainers) { 
    auto servo_it = node_->servoMap_.find(0x10);
    auto motor_it = node_->motorMap_.find(0x21);
    ASSERT_EQ(servo_it != node_->servoMap_.end(), true);
    ASSERT_EQ(motor_it != node_->motorMap_.end(), true);
}

TEST_F(TestMotorDriver, SetSinglePosition) { FAIL(); }

TEST_F(TestMotorDriver, ChangeOPMode) { FAIL(); }

TEST_F(TestMotorDriver, SetMultiplePosition) { FAIL(); }

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    RclCppFixture g_rclcppfixture(argc, argv);
    return RUN_ALL_TESTS();
}