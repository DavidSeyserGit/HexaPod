#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "invere_kinematics");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("goal_positions", 10);

    ros::Rate loop_rate(10); 

    while (ros::ok())
    {
        std_msgs::Int32MultiArray goal_pos;
        goal_pos.data.clear(); //safety measure for empty array

        //goal_pos.data.push_back(0); //motor 1
        //goal_pos.data.push_back(123); //motor 2
        //goal_pos.data.push_back(235); //motor 3
        //goal_pos.data.push_back(70); //motor 4

        pub.publish(goal_pos); // Publish the message

        ros::spinOnce();
        loop_rate.sleep(); // Sleep to maintain the loop rate
    }

    return 0;
}