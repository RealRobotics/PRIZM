# Prizm Dynamixel
A dynamixel driver that can run a configurable number of servos and motors, to be determined at boot-time.

## Topics Table

|  Parent |    Topic   |              Message              |                                                                                          Description                                                                                         |
|:-------:|:----------:|:---------------------------------:|:--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| /driver | /motor_driver/motor_vel | prizm_interfaces/msg/DynamixelVelArr | List of velocities in RPM. If the corresponding ID is associated with a servo,  it sets the profile velocity If the corresponding ID is associated with a motor,  it sets the goal velocity. |
| /driver | /motor_driver/servo_pos | prizm_interfaces/msg/DynamixelPosArr | List of positions in degrees. The servos are initialised in Extended Position Control mode, so the range is quite large.                                                                     |

## Services Table

|  Parent |               Service              |                 Interface                 |                                             Description                                            |
|:-------:|:----------------------------------:|:-----------------------------------------:|:--------------------------------------------------------------------------------------------------:|
| /driver |      /motor_driver/get_status      |  prizm_interfaces/srv/DynamixelGetStatus | Fetches current position, velocity, operating mode and torque status for a list of motor/servo IDs |
| /driver | /motor_driver/set_profile_velocity | prizm_interfaces/srv/DynamixelSetProfVel | Sets the profile velocity for a list of servos.                                                    |
| /driver |      /motor_driver/set_torque      |            std_srvs/srv/SetBool           | Sets the torque output to true or false, depending on value.                                       |


## Params Table

|  Parent | Param                              | Type   |                                                                   Description                                                                   |
|:-------:|------------------------------------|--------|:-----------------------------------------------------------------------------------------------------------------------------------------------:|
| /driver | .motor_driver.debug                | bool   | If true, all comms to the U2D2 board are ignored.                                                                                               |
| /driver | .motor_driver.device_addr          | string | COM port address of the U2D2 board.                                                                                                             |
| /driver | .motor_driver.num_motors           | int    | Number of devices that will be run in motor (velocity) mode. This must match the number of motor IDs below or an error will be thrown.          |
| /driver | .motor_driver.motor_ids            | [int]  | Individual IDs of each device to run in motor mode.                                                                                             |
| /driver | .motor_driver.num_servos           | int    | Number of devices that will be run in servo (extended position) mode. This must match the number of servo IDs below or an error will be thrown. |
| /driver | .motor_driver.servo_ids            | [int]  | Individual IDs of each device to run in servo mode.                                                                                             |
| /driver | .motor_driver.profile_acceleration | float  | Profile acceleration for the motors. This does not affect servos.                                                                               |
| /driver | .motor_driver.profile_velocity     | float  | Profile velocity for the servos. This does not affect the motors.                                                                               |
| /driver | .motor_driver.servo_max            | float  | Global max for the servo positions.                                                                                                             |
| /driver | .motor_driver.servo_min            | float  | Global min for the servo positions.                                                                                                             |
| /driver | .motor_driver.speed_max            | float  | Global max for the motor speed.                                                                                                                 |
| /driver | .motor_driver.speed_max            | float  | Global min for the motor speed                                                                                                                  |

# Author
Vittorio Francescon @ University of Leeds