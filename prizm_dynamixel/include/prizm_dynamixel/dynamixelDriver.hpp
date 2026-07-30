/**
 * @file dynamixelDriver.hpp
 * @author Vittorio Francescon (vittorio.francescon@gmail.com)
 * @brief Driver class, implemented for XC-330-T288-T Currently. Spins up some
 * ros nodes for remote control
 * @version 0.1
 * @date 27-07-2026
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <prizm_interfaces/msg/dynamixel_pos.hpp>
#include <prizm_interfaces/msg/dynamixel_pos_arr.hpp>
#include <prizm_interfaces/msg/dynamixel_vel.hpp>
#include <prizm_interfaces/msg/dynamixel_vel_arr.hpp>
#include <prizm_interfaces/srv/dynamixel_get_status.hpp>
#include <prizm_interfaces/srv/dynamixel_set_prof_vel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "prizm_dynamixel/dynamixelMotor.hpp"

// parameters len
#define GOAL_POS_LEN 4
#define GOAL_VEL_LEN 4
#define GOAL_ACCEL_LEN 4
#define TORQUE_ENABLE_LEN 1
#define LED_ENABLE_LEN 1
#define CURR_TEMP_LEN 1
#define CURR_VOLT_LEN 2
#define OP_MODE_LEN 1
#define DRIVE_MODE_LEN 1
#define PROF_VEL_LEN 4
#define CURR_POS_LEN 4
#define CURR_VEL_LEN 4

// parameters address
#define PROF_VEL_ADDR 112
#define GOAL_POS_ADDR 116
#define GOAL_VEL_ADDR 104
#define PROF_ACCEL_ADDR 108
#define TORQUE_ENABLE_ADDR 64
#define LED_ENABLE_ADDR 65
#define CURR_POS_ADDR 132
#define CURR_VEL_ADDR 128
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
typedef prizm_interfaces::srv::DynamixelSetProfVel DynaSetProfVel;
typedef prizm_interfaces::srv::DynamixelGetStatus DynaGetStatus;
typedef std_srvs::srv::SetBool SetBool;

typedef std::map<uint8_t, std::vector<uint8_t>> BulkParamMap;

class DynamixelDriver : public DynamixelMotor, public rclcpp::Node {
   public:
    // void InitServos();
    void InitMotors();
    void InitDynamixelComms();
    void InitGoalSubscribers();
    void InitServices();
    void enableTorque(std::vector<uint8_t> ids);
    void readMotorID();
    void readMotorLED();

    /**
     * @brief Sets the output torque on or off based on input.
     * This function does not close the mutex.
     *
     * @param ids list of motor IDs to toggle.
     * @param toggle true for on, false for off.
     */
    void set_torque(std::vector<uint8_t> ids, uint8_t toggle);

    /**
     * @brief Sets the output torque on or off based on input.
     * This function does not close the mutex.
     *
     * @param ids list of motor IDs to toggle.
     * @param toggle true for on, false for off.
     */
    void set_leds(std::vector<uint8_t> ids, uint8_t toggle);

    /**
     * @brief Changes operating mode based on inputted map, see
     * ```OperatingModes``` enum for options. It enables torque at the end, so
     * no need to enable after calling it.
     *
     * @param op_mode_map K: ID of the motor, V: Operating Mode code. See
     * ```OperatingModes```
     */
    void changeOperatingMode(std::map<uint8_t, uint8_t> op_mode_map);

    void setGoalVelocity(std::map<uint8_t, float> vel_map);
    void setProfileVelocity(std::map<uint8_t, float> vel_map);
    void setProfileAccel(std::map<uint8_t, float> accel_map);
    void setServoPos(std::map<uint8_t, float> pos_map);

    std::map<uint8_t, float> getPos(std::vector<uint8_t> ids);
    std::map<uint8_t, float> getVel(std::vector<uint8_t> ids);

    void servoPosGoalCB_(const DynaPosArr& msg);
    void motorVelGoalCB_(const DynaVelArr& msg);
    void setProfVelSrvCB_(const DynaSetProfVel::Request::SharedPtr req,
                          DynaSetProfVel::Response::SharedPtr res);
    void getStatusCB_(const DynaGetStatus::Request::SharedPtr req,
                      DynaGetStatus::Response::SharedPtr res);
    void setTorqueCB_(const SetBool::Request::SharedPtr req,
                      SetBool::Response::SharedPtr res);

    std::string address = "/dev/ttyUSBX";
    std::string paramNamespace_ = "motor_driver";
    int numServos_, servoMin_, servoMax_;
    int numMotors_, speedMin_, speedMax_;

    dynamixelMap servoMap_, motorMap_;

    std::vector<int64_t> motorIdsINT_, servoIdsINT_;

    std::vector<uint8_t> allIds_, motorIds_, servoIds_;
    DynamixelDriver();
    DynamixelDriver(std::string& nodeName, rclcpp::NodeOptions& options);
    ~DynamixelDriver();

    rclcpp::Subscription<DynaPosArr>::SharedPtr servoGoalPosSub_;
    rclcpp::Subscription<DynaVelArr>::SharedPtr motorGoalPosSub_;
    rclcpp::Service<DynaSetProfVel>::SharedPtr setProfVelSrv_;
    rclcpp::Service<DynaGetStatus>::SharedPtr getStatusSrv_;
    rclcpp::Service<SetBool>::SharedPtr setTorqueSrv_;

    static dynamixel::PortHandler* portHandler_;
    static dynamixel::PacketHandler* packetHandler_;
    static dynamixel::GroupBulkRead* groupBulkRead_;
    static dynamixel::GroupBulkWrite* groupBulkWrite_;

    // dynamixel::PortHandler* portHandler_;
    // dynamixel::PacketHandler* packetHandler_;
    // std::shared_ptr<dynamixel::GroupBulkRead> groupBulkRead_;
    // std::shared_ptr<dynamixel::GroupBulkWrite> groupBulkWrite_;
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    float profile_velocity = 0, profile_acceleration = 0;

   private:
    bool debug = true;
    std::string nodeName_;

    /**
     * @brief ensures that only one thread ever uses the comms channlel.
     * If you are writing a function that uses it, make sure to call
     * ```std::scoped_lock(comms_mutex)```. No need to release it.
     */
    std::mutex comms_mutex_;

    /**
     * @brief Uses ```groupBulkWrite_``` to write multiple instructions in one
     * go. This function does not have any blocking builtin. The Writing
     * functions are expected to implement their own to ensure no data races
     * occur.
     *
     * @param val_map K: motor ID to write for V: std::vector<uint8_t> a
     * uint32_t broken into a list of uint8_t. Use the conversion functions in
     * ```DynamixelMotor``` to handle this.
     * @param insturction_code Instruction address from the motor reference
     * sheet
     * @param instruction_len Length of the instruction from the reference
     * sheet.
     * @return 1 for succcessful comm, -1 for unsuccessful
     */
    int writeParams(std::map<uint8_t, std::vector<uint8_t>> val_map,
                    uint16_t insturction_code, uint16_t instruction_len);

    std::map<uint8_t, uint32_t> readParams(std::vector<uint8_t> addresses,
                                           uint16_t instruction_code,
                                           uint16_t instruction_len);

    void singleRead(uint8_t id, uint16_t code);
    void singleRead4B(uint8_t id, uint16_t code);
    void singleWrite(uint8_t id, uint16_t code, uint8_t data);
    void singleWrite4B(uint8_t id, uint16_t code, uint32_t data);
};
