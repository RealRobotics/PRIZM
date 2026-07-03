#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <dynamixel_sdk/dynamixel_sdk.h>

/**
 * @brief dynamixel data class. use this to hold all your data in a convenient way
 */
struct dynamixelData {
    uint8_t motorId_ = 0x00;

    uint32_t currentPos_ = 0x00;
    uint32_t currentVel_ = 0x00;
    uint32_t currentAccel_ = 0x00;
    uint8_t errorStatus_ = 0x00;
    bool torqueEnable_ = false;
    uint8_t operatingMode_ = 0x00;
    uint32_t goalPos_ = 0x00;
    uint32_t goalVel_ = 0x00;
    uint32_t goalAccel_ = 0x00;
    float convPos_ = 0.f;
    float convVel_ = 0.f;
    float convAccel_ = 0.f;

    std::string motorModel_;
};

/**
 * @brief template class for dynamixel motors. Inherit this and implement the
 * relevant methods.
 */
class DynamixelMotor {
   public:
    DynamixelMotor();
    DynamixelMotor(int motorId);

    dynamixelData data;

    virtual uint8_t getMotorId();
    virtual uint32_t getOpMode();
    virtual uint32_t getTemp();
    virtual uint32_t getErrorStatus();
    virtual uint32_t getCurrentPos();
    virtual uint32_t getCurrentVel();
    virtual uint32_t getCurrentAccel();

    static dynamixel::PortHandler* portHandler_;
    static dynamixel::PacketHandler* packetHandler_;
    static dynamixel::GroupBulkRead* groupBulkRead_;
    static dynamixel::GroupBulkWrite* groupBulkWrite_;
   private:
};
