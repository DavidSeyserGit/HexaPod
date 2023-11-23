#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

const std::vector<uint8_t> DXL_ID = {1, 2, 3, 4, 5, 6, 7, 8, 9};
const uint8_t ADDR_GOAL_POSITION = 30;
const float PROTOCOL_VERSION = 1.0;
const int BAUDRATE = 57600;
const char* DEVICENAME = "/dev/ttyUSB0"; 

dynamixel::PortHandler *portHandler = nullptr;
dynamixel::PacketHandler *packetHandler = nullptr;   

void motorCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    int dxl_comm_result = COMM_TX_FAIL;
    for (size_t i = 0; i < DXL_ID.size(); i++)
    {
        uint16_t dxl_goal_position = msg->data[i]; //value between 0 and 360 degrees

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_GOAL_POSITION, dxl_goal_position);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            ROS_ERROR("Failed to write goal position to Dynamixel ID");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motorcontroller");
    ros::NodeHandle nh;

    if(portHandler == nullptr){
        portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    }

    if(packetHandler == nullptr){
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    }

    portHandler->openPort();
    portHandler->setBaudRate(BAUDRATE);

    //subscriber implementation for inverse kinematics

    portHandler->closePort();

    return 0;
}