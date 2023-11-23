#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <cmath> 

namespace gait{

    typedef struct {
        double x, y, z;
    }GoalPositions;

    std::vector<GoalPositions> calculateTripodGaitPositions(double step_length, double step_height, double body_width, double cycle_progress) {
        std::vector<GoalPositions> positions(6);
        //implementation of gait control
        return positions;
    }
}

namespace inverse_kinematics{

    typedef struct {
        double theta1, theta2, theta3;
    }JointAngles;

    JointAngles calculateIK(double x, double y, double z, double l1, double l2, double l3) 
    {
        JointAngles angles;        
        
        //theta 1
        angles.theta1 = atan2(y, x);
        //theta 2
        const float l2l3xy = sqrt(pow(l2, 2) + pow(l3, 2));
        float beta = atan2(z, l2l3xy);
        float alpha = acos((pow(l2, 2) + pow(l2l3xy, 2) - pow(l3, 2)) / (2 * l2 * l2l3xy));
        angles.theta2 = alpha - fabs(beta);

        //theta 3
        float gamma = acos((pow(l3, 2) + pow(l2l3xy, 2) - pow(l2, 2)) / (2 * l3 * l2l3xy));
        angles.theta3 = gamma + fabs(beta);

        return angles;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "invere_kinematics");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int16MultiArray>("goal_positions", 10);

    ros::Rate loop_rate(10); 

    while (ros::ok()) 
    {
        std_msgs::Int16MultiArray goal_pos;
        goal_pos.data.clear();

        double x, y, z, l1, l2, l3, step_length, step_height, body_width; //placeholder, needs to be read out from node (similar to MSROB Homework 2)
        int cycle_progress = 0; // value from 0 - 1 
        
        std::vector<gait::GoalPositions> leg_positions = gait::calculateTripodGaitPositions(step_length, step_height, body_width, cycle_progress);

        for (size_t i = 0; i < leg_positions.size(); ++i) {
            inverse_kinematics::JointAngles angles = inverse_kinematics::calculateIK(leg_positions[i].x, leg_positions[i].y, leg_positions[i].z, l1, l2, l3);

        goal_pos.data.push_back(static_cast<int16_t>(angles.theta1));
        goal_pos.data.push_back(static_cast<int16_t>(angles.theta2));
        goal_pos.data.push_back(static_cast<int16_t>(angles.theta3));
        pub.publish(goal_pos);
        }
    }   
    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}