#include <dynamixel_sdk/dynamixel_sdk.h>

#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>

#define ANGLE_TO_TICK 0.088
#define SERVO_FULL_ROT 4096
#define MOTOR_MAX_RPM 73.24
#define MOTOR_MAX_PVE 320
// #define MOTOR_MAX_NVE 0xFFFFFEC0

/**
 * @brief dynamixel data class. use this to hold all your data in a convenient
 * way
 */
struct dynamixelData {
    uint8_t motorId_ = 0x00;

    uint32_t currentPos_ = 0x00;
    uint32_t currentVel_ = 0x00;
    uint32_t currentAccel_ = 0x00;
    uint8_t errorStatus_ = 0x00;
    uint32_t currentVoltage = 0x00;
    uint32_t temp_ = 0x00;
    bool torqueEnable_ = false;
    uint8_t operatingMode_ = 0x00;
    uint8_t driveMode_ = 0x00;
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
    DynamixelMotor() { data = dynamixelData(); }
    DynamixelMotor(uint8_t motorId) {
        data = dynamixelData();
        data.motorId_ = motorId;
    }

    dynamixelData data;

    virtual uint8_t getMotorId() { return data.motorId_; }
    virtual uint32_t getOpMode() { return data.operatingMode_; }
    virtual uint32_t getTemp() { return data.temp_; }
    virtual uint32_t getErrorStatus() { return data.errorStatus_; }
    virtual uint32_t getCurrentPos() { return data.currentPos_; }
    virtual uint32_t getCurrentVel() { return data.currentVel_; }
    virtual uint32_t getCurrentAccel() { return data.currentAccel_; }
    virtual uint32_t getVoltage() { return data.currentVoltage; }

    virtual uint8_t setPosGoal(uint32_t goal_pos) {
        data.goalPos_ = goal_pos;
        return 1;
    }
    virtual uint8_t setVelGoal(uint32_t goal_vel) {
        data.goalVel_ = goal_vel;
        return 1;
    }
    virtual uint8_t setAccelGoal(uint32_t goal_accel) {
        data.goalAccel_ = goal_accel;
        return 1;
    }

    virtual uint8_t toggleTorqueMode(bool torqueMode) {
        data.torqueEnable_ = torqueMode;
        return 1;
    }

    virtual uint8_t setOpMode(uint8_t op_mode) {
        data.operatingMode_ = op_mode;
        return 1;
    }

    virtual uint8_t setDriveMode(uint8_t drive_mode) {
        data.driveMode_ = drive_mode;
        return 1;
    }

    uint8_t* degToPos(float angle_deg) {
        int target_pos =
            static_cast<uint16_t>(angle_deg / 360 * SERVO_FULL_ROT);
        uint8_t goal_pos[4];
        goal_pos[0] = DXL_LOBYTE(DXL_LOWORD(target_pos));
        goal_pos[1] = DXL_HIBYTE(DXL_LOWORD(target_pos));
        goal_pos[2] = DXL_LOBYTE(DXL_HIWORD(target_pos));
        goal_pos[3] = DXL_HIBYTE(DXL_HIWORD(target_pos));
        return goal_pos;
    }

    float posToDeg(uint16_t pos) {
        return static_cast<float>(pos / SERVO_FULL_ROT * 360);
    }

    uint8_t* rpmToHex(float vel) {
        uint8_t goal_vel[4];
        uint32_t vel_hex;
        if (vel < 0) {
            vel_hex =
                ~static_cast<uint32_t>(vel / MOTOR_MAX_RPM * MOTOR_MAX_PVE) + 1;

        } else {
            // static_cast<uint32_t>();
            vel_hex =
                static_cast<uint32_t>(vel / MOTOR_MAX_RPM * MOTOR_MAX_PVE);
        }
        goal_vel[0] = DXL_LOBYTE(DXL_LOWORD(vel_hex));
        goal_vel[1] = DXL_HIBYTE(DXL_LOWORD(vel_hex));
        goal_vel[2] = DXL_LOBYTE(DXL_HIWORD(vel_hex));
        goal_vel[3] = DXL_HIBYTE(DXL_HIWORD(vel_hex));
        return goal_vel;
    }

    // dynamixel::PortHandler* portHandler_;
    // dynamixel::PacketHandler* packetHandler_;
    // std::shared_ptr<dynamixel::GroupBulkRead> groupBulkRead_;
    // std::shared_ptr<dynamixel::GroupBulkWrite> groupBulkWrite_;

   private:
};
