#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "hitec-can-servo-reg-public.h"
#include "prizm_interfaces/msg/claw_extension.hpp"
#include "prizm_interfaces/msg/hitech_angle.hpp"
#include "prizm_interfaces/srv/set_hitech_angle.hpp"
#include "prizm_interfaces/srv/set_stream_registers.hpp"

// typedef prizm_interfaces::msg::ClawExtension ClawExtension;
// typedef prizm_interfaces::msg

enum RunMode {
    MULTI_TURN = 0,           // multi quadrant mode
    SERVO_MODE = 1,           // one quadrant
    CONTINUOUS_ROTATION = 2,  // needs direction and output
    SPEED = 3  // Experimental. needs direction and autocalcs output
};

enum WriteTasks {
    NEW_POSITION = e_POSITION_NEW,
    NEW_TURN = e_TURN_NEW,
    SERVO_MAX = e_POSITION_MAX,
    SERVO_MIN = e_POSITION_MIN,
    SERVO_ZERO = e_POSITION_MID,
    NEW_SPEED = e_VELOCITY,
    STREAM_MODE = e_STREAM_MODE,  // 0=off, 1=on
    STREAM_TIME = e_STREAM_TIME,  // ms
    REG_STREAM_0 = e_STREAM_0,
    REG_STREAM_1 = e_STREAM_1,
    REG_STREAM_2 = e_STREAM_2,
    REG_STREAM_3 = e_STREAM_3,

    ERROR = 0xFF
};

enum ReadTasks {
    POSITION = 0x0C,        // 4095 = 90deg
    TURN = e_TURN_COUNT,    // 1 = 360deg
    VOLTAGE = e_VOLTAGE,    // 100 = 1.0V
    TORQUE = e_TORQUE,      // PWM out. 4095 = 100%
    VELOCITY = e_VELOCITY,  // posdiff/100msec
    /**
     * @brief
     * Bit6:    Volt Very low
     * Bit7:    Over Volt Break
     * Bit8:    Pos Emg Min under
     * Bit9:    Pos Emg Max over
     * Bit10:   MCU Temp under
     * Bit11:   MCU Temp over
     * Bit13:   Voltage Under
     * Bit14:   Voltage Over
     */
    E_STOP = e_EMERGENCY_STOP
};

struct hitechData {
    uint8_t run_mode = SERVO_MODE;
    uint16_t currAngle = 0x00;  // Range 0 -> 360. 4096 = 90deg
    int16_t currTurn = 0x00;    // 1 = 360deg
    uint16_t currPos = 0x00;    // currTurn * 360 + currAngle
    uint16_t goalAngle = 0x00;  // Range 0 -> 360. 4096 = 90deg
    int16_t goalTurn = 0x00;    // 1 = 360deg
    uint16_t goalPos = 0x00;    // currTurn * 360 + currAngle
    uint16_t currVel = 0x00;
    uint16_t servoMin = 0x0000;
    uint16_t servoMax = 0x4000;
    uint16_t servoZero = 0x2000;
    uint16_t voltage = 0x00;  // 100 = 1.0V
    uint16_t torque = 0x00;   // PWM. 4095 = 100%
    uint8_t stream_toggle = false;
    uint16_t stream_time = 0x00;  // Interval to set. 0 to 10000 ms, 10001 to
                                  // 11000 = (Value-10000) Hz
    uint8_t stream_00 = 0x00;
    uint8_t stream_01 = 0x00;
    uint8_t stream_10 = 0x00;
    uint8_t stream_11 = 0x00;
    uint8_t stream_20 = 0x00;
    uint8_t stream_21 = 0x00;
    uint8_t stream_30 = 0x00;
    uint8_t stream_31 = 0x00;

    uint16_t e_stop = 0x00;
};

typedef std::map<uint8_t, hitechData> hitechMap;
typedef std_msgs::msg::UInt8MultiArray uint8Arr;
typedef prizm_interfaces::msg::HitechAngle HitechAngle;
typedef prizm_interfaces::srv::SetHitechAngle SetHitechAngle;
typedef prizm_interfaces::srv::SetStreamRegisters SetStreamRegisters;

class HitechBindings : public rclcpp::Node {
   private:
    // initialisers
    void initContainers();
    void initParameters();
    void initSubscribers();
    void initPublishers();
    void initServices();

    // setters
    /**
     * @brief Takes in parsed received data and populates the relevant structs
     * with the obtained info.
     *
     * @param servo_id servo id to update
     * @param addr register the data came from
     * @param data data repacked into a single int
     */
    void readHandler(uint8_t servo_id, uint8_t addr, uint16_t data);

    /**
     * @brief Set the Stream Time for a servo and publish it downstream
     *
     * @param servo_id servo id to operate on
     * @param stream_time Interval to set. 0 to 10000 ms, 10001 to 11000 =
     * (Value-10000) Hz
     */
    void setStreamTime(uint8_t servo_id, int stream_time);

    /**
     * @brief Toggle the Stream Mode for a servo and publish downstream.
     *
     * @param servo_id servo id to operate on
     * @param toggle True: stream at frequency ```HitechData::stream_time```
     */
    void toggleStreamMode(uint8_t servo_id, bool toggle);

    /**
     * @brief Set a pair of streaming registers for a given servo id
     *
     * @param servo_id servo id to operate on
     * @param stream_addr See ```WriteTasks::REG_STREAM_0/1/2/3```.
     * @param reg1 First address to stream. See ```ReadTasks``` for valid inputs
     * @param reg2 Second address to stream. See ```ReadTasks``` for valid
     * inputs
     */
    void setStreamReg(uint8_t servo_id, WriteTasks stream_addr, uint8_t reg1,
                      uint8_t reg2);

    // getters

    // attributes
    uint8_t canProtocol_ = 0x01;  // make this configurable
    int numServos_, numMotors_;
    std::string paramNamespace_ = "hitech_driver";
    std::vector<int64_t> servoMins_, servoMaxs_, servoZeros_;
    std::vector<int64_t> speedMins_, speedMaxs_;
    std::vector<int64_t> servoIds_, motorIds_;
    // publishers
    rclcpp::Publisher<HitechAngle>::SharedPtr anglePub_;
    rclcpp::Publisher<uint8Arr>::SharedPtr serialPub_;

    // subscribers
    rclcpp::Subscription<uint8Arr>::SharedPtr serialSub_;

    // services
    rclcpp::Service<SetHitechAngle>::SharedPtr setAngleSrv_;
    rclcpp::Service<SetStreamRegisters>::SharedPtr setStreamSrv_;

    // clients

    // callbacks
    void serialReadCB_(const uint8Arr& msg);
    void setAngleCB_(const SetHitechAngle::Request::SharedPtr req,
                     SetHitechAngle::Response::SharedPtr res);
    void setStreamCB_(const SetStreamRegisters::Request::SharedPtr req,
                      SetStreamRegisters::Response::SharedPtr res);

   public:
    HitechBindings() : Node("HitechBindings") {}
    HitechBindings(std::string& nodeName, rclcpp::NodeOptions& options);

    void setServoPos(int id, int target_angle);
    // helpers
    std::vector<uint8_t> singleWriteBuilder(WriteTasks task, int id,
                                            int target);
    std::vector<uint8_t> doubleWriteBuilder(WriteTasks task1, WriteTasks task2,
                                            int id, int target1, int target2);
    int singleReadBuilder();
    int doubleReadBuilder();

    void singleReadHandler(const uint8Arr& msg);
    void doubleReadHandler(const uint8Arr& msg);

    void publishServoInfo();
    void publishSerialCommand(std::vector<uint8_t> message);

    WriteTasks WriteTaskfromUint(uint8_t val);
    // structs
    hitechMap servoMap_, motorMap_;
};
