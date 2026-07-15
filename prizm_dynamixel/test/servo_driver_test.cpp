#include <gtest/gtest.h>

#include <prizm_dynamixel/dynamixelServoDriver.hpp>

class TestServoMotorDriver : public ::testing::Test
{
    protected:

    void SetUp();
    void spin();
    void TearDown();

    std::shared_ptr<DynamixelServoDriver> node_;
    rclcpp::executors::SingleThreadedExecutor * exec_;
    std::thread * thread_;
};

void TestServoMotorDriver::SetUp(){
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<DynamixelServoDriver>();
}


void TestServoMotorDriver::TearDown(){
       
}

TEST_F(TestServoMotorDriver, AddressSetting){
    SUCCEED();
}


int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}