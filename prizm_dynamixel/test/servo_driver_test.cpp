#include <gtest/gtest.h>

#include <prizm_dynamixel/dynamixelDriver.hpp>

class TestMotorDriver : public ::testing::Test {
   protected:
    void SetUp();
    void spin();
    void TearDown();

    std::shared_ptr<DynamixelDriver> node_;

   private:
    std::string nodeName = "motor_driver_test";
    std::string paramNamespace_ = "motor_driver";
};

void TestMotorDriver::SetUp() {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    std::vector<uint8_t> servoIds = {0x00, 0x01, 0x02};
    std::vector<uint8_t> motorIds = {0x06, 0x09, 0x04};

    options.parameter_overrides().emplace_back(paramNamespace_ + ".device_addr",
                                               "/dev/ttyUSB2");
    options.parameter_overrides().emplace_back(paramNamespace_ + ".num_servos", 3);
    options.parameter_overrides().emplace_back(paramNamespace_ + ".num_motors", 3);
    options.parameter_overrides().emplace_back(paramNamespace_ + ".servo_ids", servoIds);
    options.parameter_overrides().emplace_back(paramNamespace_ + ".motor_ids", motorIds);
    

    node_ = std::make_shared<DynamixelDriver>(nodeName, options);
}

void TestMotorDriver::TearDown() {
    node_.reset();
    rclcpp::shutdown();
}

TEST_F(TestMotorDriver, AddressSetting) {
    ASSERT_EQ("/dev/ttyUSB2", node_->address);
}

TEST_F(TestMotorDriver, InitMotors) { 
    ASSERT_EQ(node_->servoList_[2].motorId_, 0x02);
    ASSERT_EQ(node_->motorList_[1].motorId_, 0x09);
}

TEST_F(TestMotorDriver, InitServoMismatch) { FAIL(); }
TEST_F(TestMotorDriver, InitMotorsMismatch) { FAIL(); }

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}