#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <cmath> 

#define LEG_COUNT_FOR_GAIT 3

namespace gait {

    typedef struct {
        double x, y, z;
    } GoalPositions;

    void calculateTripodGaitPositions(GoalPositions positions[LEG_COUNT_FOR_GAIT], double step_length, double step_height, double body_width, double cycle_progress) {
        // implementation of gait control
        // needs to set cycle_state on change (first set of legs reach goal_pos)
    }
}


namespace inverse_kinematics {

    typedef struct {
        double theta1, theta2, theta3;
    } JointAngles;

    const double l1 = 0, l2 = 0, l3 = 0; //need to be changed to real values from value (PLACEHOLDER)
    
    JointAngles calculateIK(double goal_x, double goal_y, double goal_z) // not goal_pos in worldframe but goal_pos of the leg_endeffector
    {
        JointAngles angles;        
        
        //theta 1
        angles.theta1 = atan2(goal_y, goal_x);

        //theta 2
        const float l2l3xy = sqrt(pow(l2, 2) + pow(l3, 2));
        float beta = atan2(goal_z, l2l3xy);
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
        std_msgs::Int16MultiArray joint_variables;

        /*
        joint_variables.data should only happen 3 times. all angles are known for one leg per function call
        LEG 1 Inverese Kinematics

        First pushback needs to be gait_state
        */

        joint_variables.data.push_back(1); //GAIT

        joint_variables.data.push_back(static_cast<int16_t>(inverse_kinematics::calculateIK(0.1, 0.1, 0.1).theta1)); //goal_pos needs to be changed for every leg
        joint_variables.data.push_back(static_cast<int16_t>(inverse_kinematics::calculateIK(0.1, 0.1, 0.1).theta2));
        joint_variables.data.push_back(static_cast<int16_t>(inverse_kinematics::calculateIK(0.1, 0.1, 0.1).theta3));
        //LEG 2 Inverese Kinematics
        joint_variables.data.push_back(static_cast<int16_t>(inverse_kinematics::calculateIK(0.1, 0.1, 0.1).theta1)); //goal_pos needs to be changed for every leg
        joint_variables.data.push_back(static_cast<int16_t>(inverse_kinematics::calculateIK(0.1, 0.1, 0.1).theta2));
        joint_variables.data.push_back(static_cast<int16_t>(inverse_kinematics::calculateIK(0.1, 0.1, 0.1).theta3));
        //LEG 3 Inverese Kinematics
        joint_variables.data.push_back(static_cast<int16_t>(inverse_kinematics::calculateIK(0.1, 0.1, 0.1).theta1)); //goal_pos needs to be changed for every leg
        joint_variables.data.push_back(static_cast<int16_t>(inverse_kinematics::calculateIK(0.1, 0.1, 0.1).theta2));
        joint_variables.data.push_back(static_cast<int16_t>(inverse_kinematics::calculateIK(0.1, 0.1, 0.1).theta3));
        //value between 0 and 360 degrees

        joint_variables.data.clear();

        pub.publish(joint_variables);
    }   
    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}