#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#include "Task3.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Task3::Task3()
{
    double angle = PI/2;
    _corridorEnd.target_pose.header.frame_id = "marrtino_map";
    _corridorEnd.target_pose.pose.position.x = -1.16;
    _corridorEnd.target_pose.pose.position.y = 3.10;
    _corridorEnd.target_pose.pose.orientation.x = 0.0;
    _corridorEnd.target_pose.pose.orientation.y = 0.0;
    _corridorEnd.target_pose.pose.orientation.z = sin(angle/2);
    _corridorEnd.target_pose.pose.orientation.w = cos(angle/2);
};

bool Task3::init(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    return true;
}

void Task3::run()
{
    ros::NodeHandle n;
    _corridorEnd.target_pose.header.stamp = ros::Time::now();

    ros::Publisher motor_control = n.advertise<geometry_msgs::Twist>(MOTOR_TOPIC, Q_LEN);

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) 
        ROS_INFO("Waiting for the move_base action server"); 

    ROS_INFO("Moving to the corridor end.");
    ac.sendGoal(_corridorEnd);
    ac.waitForResult();

    if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("Failed to reach goal position.");
        return;
    }

    _turn(-PI/2, 0.5, motor_control);
}

void Task3::_turn(float angle, double speed, ros::Publisher& motor_control)
{
    ros::Rate rate(10);
    int ticks = 0;
    int endTicks = static_cast<int>(angle/speed) + 1;
    while(ticks < endTicks)
    {
        _set_velocities(0.0, speed, motor_control);
        ros::spinOnce();
        rate.sleep();
    }
    _set_velocities(0.0, 0.0, motor_control);
}

void Task3::_set_velocities(float lin_vel, float ang_vel, ros::Publisher& motor_control)
{
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;

    motor_control.publish(msg);
}