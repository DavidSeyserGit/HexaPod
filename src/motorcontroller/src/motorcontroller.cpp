#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

/*
leerlauf strom rasppi + 4 beine:
    0.5-0.7A

Strom raspPi mit 8 Motoren Haltemoment
    0.95A

Laststrom 8 Motoren + rasppi
    2.2A
*/  

// Dynamixel settings

#define DXL_ID_1            14
#define DXL_ID_2            16
#define DXL_ID_3            18

#define DXL_ID_4            7
#define DXL_ID_5            9
#define DXL_ID_6            11

#define DXL_ID_7            1
#define DXL_ID_8            3
#define DXL_ID_9            5
/*
#define DXL_ID_10           18
#define DXL_ID_11           10
#define DXL_ID_12           12

#define DXL_ID_7            8
#define DXL_ID_8            14
#define DXL_ID_9            2
#define DXL_ID_10           1
#define DXL_ID_11           13
#define DXL_ID_12           17
*/

#define PROTOCOL_VERSION    1.0
#define BAUDRATE            1000000
#define DEVICENAME          "/dev/ttyUSB0"


// Control table addresses for Dynamixel AX-12A
#define ADDR_AX12A_GOAL_POSITION   30
#define DXL_MOVING_STATUS_THRESHOLD 20


dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
dynamixel::GroupSyncWrite *groupSyncWrite;

void initializeDynamixel() {

    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    if (portHandler->openPort() && portHandler->setBaudRate(BAUDRATE)) {
        // Enable Dynamixel torque for each motor
        for (int dxl_id = DXL_ID_1; dxl_id <= DXL_ID_9; ++dxl_id) {
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

void moveToPositions(const std::vector<uint8_t>& dxl_ids, const std::vector<int>& goal_positions) {
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

    initializeDynamixel();

    std::vector<uint8_t> dxl_ids = {DXL_ID_1, DXL_ID_2, DXL_ID_3, DXL_ID_4, DXL_ID_5, DXL_ID_6, DXL_ID_7, DXL_ID_8, DXL_ID_9};
    std::vector<int> goal_positions1 = {600, 400, 512, 300, 600, 512, 450, 600, 512};
    std::vector<int> goal_positions2 = {512, 700, 400, 400, 400, 512, 512, 400, 512};
    std::vector<int> goal_positions3 = {400, 400, 512, 512, 600, 512, 650, 600, 512};

    groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_AX12A_GOAL_POSITION, 2);
    if (groupSyncWrite == nullptr) {
        ROS_ERROR("Failed to allocate memory for GroupSyncWrite");
        return -1;
    }
    ros::Rate rate(1);  // 1Hz

    while (ros::ok()) {
        // Move the motors to the specified positions
        moveToPositions(dxl_ids, goal_positions1);
        rate.sleep();
        rate.sleep();
        moveToPositions(dxl_ids, goal_positions2);
        rate.sleep();
        rate.sleep();
        moveToPositions(dxl_ids, goal_positions3);
        rate.sleep();
        rate.sleep();


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
