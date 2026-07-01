#include "prizm_dynamixel/single_servo.hpp"

int main(int argc, char* argv[]){
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); // your dxl port name
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0); //protocol version

    portHandler->openPort();
    portHandler->setBaudRate(57600);

    return 1;
}