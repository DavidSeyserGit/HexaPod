#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <std_msgs/Float32MultiArray.h>


#define DXL_ID_1            14
#define DXL_ID_2            16
#define DXL_ID_3            18


#define PROTOCOL_VERSION    1.0
#define BAUDRATE            1000000
#define DEVICENAME          "/dev/ttyUSB0"


// Control table addresses for Dynamixel AX-12A
#define ADDR_AX12A_GOAL_POSITION   30
#define DXL_MOVING_STATUS_THRESHOLD 20


dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
dynamixel::GroupSyncWrite *groupSyncWrite;

std::vector<float> goal_positions;
void positionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    goal_positions = msg->data;
}

void initializeDynamixel() {

    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    if (portHandler->openPort() && portHandler->setBaudRate(BAUDRATE)) {
        // Enable Dynamixel torque for each motor
        for (int dxl_id = DXL_ID_1; dxl_id <= DXL_ID_3; ++dxl_id) {
            uint8_t dxl_error = 0;
            int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 1, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                ROS_ERROR_STREAM("Error setting torque enable for DXL_ID " << static_cast<int>(dxl_id) << ": " << packetHandler->getTxRxResult(dxl_comm_result));
                ROS_ERROR_STREAM("Dynamixel error code: " << static_cast<int>(dxl_error));
                ros::shutdown();
            }
        }
    } else {
        ROS_ERROR("Failed to open the port or set the baudrate");
        ros::shutdown();
    }
}

void moveToPositions(const std::vector<uint8_t>& dxl_ids, const std::vector<float>& goal_positions) {
    for (size_t i = 0; i < dxl_ids.size(); ++i) {
        uint8_t param_goal_position[2] = {
            DXL_LOBYTE(goal_positions[i]),
            DXL_HIBYTE(goal_positions[i])
        };

        groupSyncWrite->addParam(dxl_ids[i], param_goal_position);
    }

    // Execute the synchronized write operation
    groupSyncWrite->txPacket();
    groupSyncWrite->clearParam();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("goal", 10, positionCallback);

    initializeDynamixel();

    std::vector<uint8_t> dxl_ids = {DXL_ID_1};


    groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_AX12A_GOAL_POSITION, 2);
    ros::Rate rate(1);  // 1Hz

    while (ros::ok()) {
        // Move the motors to the specified positions
        moveToPositions(dxl_ids, goal_positions);

        ros::spinOnce();
        rate.sleep();
    }

    // Disable Dynamixel torque for each motor before exiting
    for (const auto& dxl_id : dxl_ids) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR_STREAM("Error setting torque disable for DXL_ID " << static_cast<int>(dxl_id) << ": " << packetHandler->getTxRxResult(dxl_comm_result));
        }
    }

    delete groupSyncWrite;  // Release the memory used by GroupSyncWrite

    return 0;
}
