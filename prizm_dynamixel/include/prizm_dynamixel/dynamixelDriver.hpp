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
    /**
     * @brief Sets up all the motor related stuff based on the number of
     * servos/motors and their ids that have been pulled from the config file.
     * If using this on hardware, make sure that
     * 1. ```InitDynamixelComms``` has been called first
     * 2. ```debug``` parameter has been set
     */
    void InitMotors();

    /**
     * @brief Sets up the dynamixel communication objects.
     * Namely, ```portHandler_```, ```packetHandler_```, ```groupBulkRead_```
     * and ```groupBulkWrite_```.
     * This HAS to be called before any read/write can happen.
     * The function does nothing if the parameter ```debug``` is set to true.
     */
    void InitDynamixelComms();

    /**
     * @brief Sets up the ServoPosition and MotorVelocity callbacks.
     *
     */
    void InitGoalSubscribers();

    /**
     * @brief Sets up the SetProfileVelocity, GetStatus and SetTorque services.
     *
     */
    void InitServices();

    /**
     * @brief Enables the torque for the all ids in the list provided.
     *
     * @param ids list of motor ids.
     */
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

    /**
     * @brief Set the Goal Velocity for a list of motors.
     * This function checks the motor map, so non-valid IDs are automatically
     * ignored.
     * @param vel_map K: ID of the motor V: Desired goal velocity in RPM. Will
     * be converted down the line
     */
    void setGoalVelocity(std::map<uint8_t, float> vel_map);

    /**
     * @brief Set the Profile Velocity for a list of motors.
     * This function checks the servo map, so non-valid IDs are automatically
     * ignored.
     * @param vel_map K: ID of the motor V: Desired goal velocity in RPM. Will
     * be converted down the line
     */
    void setProfileVelocity(std::map<uint8_t, float> vel_map);
    void setProfileAccel(std::map<uint8_t, float> accel_map);
    void setServoPos(std::map<uint8_t, float> pos_map);

    /**
     * @brief Get the Pos of a group of motors/servos. This function calls
     * ```readParams``` for the current position and does the hex to position
     * conversion using ```posToDeg```.
     *
     * @param ids list of motor IDs. The function does not validate this.
     * @return std::map<uint8_t, float> K: the motor/servo ID, V: the position
     * in degrees.
     */
    std::map<uint8_t, float> getPos(std::vector<uint8_t> ids);

    /**
     * @brief Get the velocity of a group of motors. This function calls
     * ```readParams``` for the current velocity and does the hex to position
     * conversion using ```hexToRpm```. This function will return 0 for servos
     * since they use the profile velocity as their velocity parameter.
     *
     * @param ids list of motor IDs. The function does not validate this.
     * @return std::map<uint8_t, float> K: the motor ID, V: the velocity in RPM.
     */
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
    int numServos_;
    int numMotors_, speedMin_, speedMax_;

    dynamixelMap servoMap_, motorMap_;

    std::vector<int64_t> motorIdsINT_, servoIdsINT_, servoMins_, servoMaxs_,
        servoZeros_;

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
     * go. This function locks the mutex, there is no need for any functions
     * using it to place their own lock.
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

    /**
     * @brief Uses ```groupBulkRead_``` to read multiple addresses at once.
     * This function locks the mutex, there is no need to lock before calling.
     *
     * @param addresses list of addresses to read. It is assumed that if you got
     * this far in the function, the list is validated.
     * @param instruction_code address to perform the read at
     * @param instruction_len byte len of the instruction
     * @return std::map<uint8_t, uint32_t> K: the ID for the read value, V: the
     * read value.
     */
    std::map<uint8_t, uint32_t> readParams(std::vector<uint8_t> addresses,
                                           uint16_t instruction_code,
                                           uint16_t instruction_len);

    /**
     * @brief utility function. Reads a single byte instruction for a single
     * motor ID. This function locks the mutex.
     *
     * @param id the ID to read from, the function does not validate this, so
     * make sure it is valid prior to calling.
     * @param code instruction address to read from
     * @return uint8_t the read value
     */
    uint8_t singleRead(uint8_t id, uint16_t code);

    /**
     * @brief utility function. Reads a 4 byte instruction for a single
     * motor ID. This function locks the mutex.
     *
     * @param id the ID to read from, the function does not validate this, so
     * make sure it is valid prior to calling.
     * @param code instruction address to read from
     * @return uint8_t the read value
     */
    void singleRead4B(uint8_t id, uint16_t code);

    /**
     * @brief utility function. Writes a 1 byte instruction for a single motor
     * id. This function locks the mutex.
     *
     * @param id the ID to write to. This function does not validate the ID, do
     * that beforehand.
     * @param code instruction code
     * @param data data to write.
     */
    void singleWrite(uint8_t id, uint16_t code, uint8_t data);

    /**
     * @brief utility function. Writes a 4 byte instruction for a single motor
     * id. This function locks the mutex.
     *
     * @param id the ID to write to. This function does not validate the ID, do
     * that beforehand.
     * @param code instruction code
     * @param data data to write.
     */
    void singleWrite4B(uint8_t id, uint16_t code, uint32_t data);
};
