#include <rclcpp/rclcpp.hpp>

#include "hitec-can-servo-reg-public.h"
#include "prizm_interfaces/msg/claw_extension.hpp"
#include "prizm_interfaces/msg/hitech_angle.hpp"

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
};

typedef std::map<uint8_t, hitechData> hitechMap;

class HitechBindings : public rclcpp::Node {
   private:
    // initialisers
    void initContainers();
    void initParameters();

    // setters

    // getters

    // attributes
    uint8_t canProtocol_ = 0x01;  // make this configurable
    int numServos_, numMotors_;
    std::string paramNamespace_ = "hitech_driver";
    std::vector<int64_t> servoMins_, servoMaxs_, servoZeros_;
    std::vector<int64_t> speedMins_, speedMaxs_;
    std::vector<int64_t> servoIds_, motorIds_;
    // publishers

    // subscribers

    // services

    // clients

    // callbacks

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

    hitechMap servoMap_, motorMap_;
};
