#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "hitec-can-servo-reg-public.h"
#include "prizm_interfaces/msg/claw_extension.hpp"
#include "prizm_interfaces/msg/hitech_angle.hpp"
#include "prizm_interfaces/msg/hitech_position.hpp"
#include "prizm_interfaces/srv/detail/set_hitech_angle__struct.hpp"
#include "prizm_interfaces/srv/set_hitech_angle.hpp"

#include "prizm_interfaces/srv/detail/set_hitech_position__struct.hpp"
#include "prizm_interfaces/srv/set_hitech_position.hpp"

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
    NEW_SPEED = e_VELOCITY
};

enum ReadTasks {
    POSITION = 0x0C,
    TURN = e_TURN_COUNT,
    VOLTAGE = e_VOLTAGE,
    TORQUE = e_TORQUE
};

struct hitechData {
    uint8_t run_mode = SERVO_MODE;
    uint16_t currAngle = 0x00;  // Range 0 -> 360. 4096 = 90deg
    int16_t currTurn = 0x00;    // 1 = 360deg
    uint16_t currPos = 0x00;    // currTurn * 360 + currAngle
    uint16_t goalAngle = 0x00;  // Range 0 -> 360. 4096 = 90deg
    int16_t goalTurn = 0x00;    // 1 = 360deg
    uint16_t goalPos = 0x00;    // currTurn * 360 + currAngle
    uint16_t servoMin = 0x0000;
    uint16_t servoMax = 0x4000;
    uint16_t servoZero = 0x2000;
    uint16_t voltage = 0x00;  // 100 = 1.0V
    uint16_t torque = 0x00;   // PWM. 4095 = 100%
};

typedef std::map<uint8_t, hitechData> hitechMap;
typedef std_msgs::msg::UInt8MultiArray uint8Arr;
typedef prizm_interfaces::msg::HitechAngle HitechAngle;
typedef prizm_interfaces::msg::HitechPosition HitechPosition;
typedef prizm_interfaces::srv::SetHitechAngle SetHitechAngle;
typedef prizm_interfaces::srv::SetHitechPosition SetHitechPosition;

class HitechBindings : public rclcpp::Node {
   private:
    // initialisers
    void initContainers();
    void initParameters();
    void initSubscribers();
    void initPublishers();
    void initServices();

    // setters
    void readHandler(uint8_t servo_id, uint8_t addr, uint16_t data);
    void updateServoPos();

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
    rclcpp::Publisher<HitechPosition>::SharedPtr positionPub_;

    // subscribers
    rclcpp::Subscription<uint8Arr>::SharedPtr serialSub_;

    // services
    rclcpp::Service<SetHitechAngle>::SharedPtr setAngleSrv_;
    rclcpp::Service<SetHitechPosition>::SharedPtr setPositionSrv_;

    // clients

    // callbacks
    void serialReadCB_(const uint8Arr& msg);
    void setAngleCB_(const SetHitechAngle::Request::SharedPtr req,
                     SetHitechAngle::Response::SharedPtr res);
    void setPositionCB_(const SetHitechPosition::Request::SharedPtr req,
                     SetHitechPosition::Response::SharedPtr res);
    

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
    // structs
    hitechMap servoMap_, motorMap_;
};
