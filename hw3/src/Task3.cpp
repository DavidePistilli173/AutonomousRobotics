#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "Task3.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Task3::Task3()
{
    double angle = PI/2;
    _corridorEnd.target_pose.header.frame_id = "marrtino_map";
    _corridorEnd.target_pose.pose.position.x = -1.11;
    _corridorEnd.target_pose.pose.position.y = 3.155;
    _corridorEnd.target_pose.pose.orientation.x = 0.0;
    _corridorEnd.target_pose.pose.orientation.y = 0.0;
    _corridorEnd.target_pose.pose.orientation.z = sin(angle/2);
    _corridorEnd.target_pose.pose.orientation.w = cos(angle/2);

    angle = 0;
    _entrance.target_pose.header.frame_id = "marrtino_map";
    _entrance.target_pose.pose.position.x = -0.45;
    _entrance.target_pose.pose.position.y = 3.15;
    _entrance.target_pose.pose.orientation.x = 0.0;
    _entrance.target_pose.pose.orientation.y = 0.0;
    _entrance.target_pose.pose.orientation.z = sin(angle/2);
    _entrance.target_pose.pose.orientation.w = cos(angle/2);

    angle = 0;
    _alternativeEntrance.target_pose.header.frame_id = "marrtino_map";
    _alternativeEntrance.target_pose.pose.position.x = 0.5;
    _alternativeEntrance.target_pose.pose.position.y = 3.155;
    _alternativeEntrance.target_pose.pose.orientation.x = 0.0;
    _alternativeEntrance.target_pose.pose.orientation.y = 0.0;
    _alternativeEntrance.target_pose.pose.orientation.z = sin(angle/2);
    _alternativeEntrance.target_pose.pose.orientation.w = cos(angle/2);

    angle = -PI/2;
    _preDockingStation1.target_pose.header.frame_id = "marrtino_map";
    _preDockingStation1.target_pose.pose.position.x = 0.17;
    _preDockingStation1.target_pose.pose.position.y = 1.0;
    _preDockingStation1.target_pose.pose.orientation.x = 0.0;
    _preDockingStation1.target_pose.pose.orientation.y = 0.0;
    _preDockingStation1.target_pose.pose.orientation.z = sin(angle/2);
    _preDockingStation1.target_pose.pose.orientation.w = cos(angle/2);

    angle = -3*PI/2;
    _preDockingStation2.target_pose.header.frame_id = "marrtino_map";
    _preDockingStation2.target_pose.pose.position.x = 0.17;
    _preDockingStation2.target_pose.pose.position.y = 1.0;
    _preDockingStation2.target_pose.pose.orientation.x = 0.0;
    _preDockingStation2.target_pose.pose.orientation.y = 0.0;
    _preDockingStation2.target_pose.pose.orientation.z = sin(angle/2);
    _preDockingStation2.target_pose.pose.orientation.w = cos(angle/2);

    angle = -PI/2;
    _dockingStation1.target_pose.header.frame_id = "marrtino_map";
    _dockingStation1.target_pose.pose.position.x = 0.17;
    _dockingStation1.target_pose.pose.position.y = 0.645;
    _dockingStation1.target_pose.pose.orientation.x = 0.0;
    _dockingStation1.target_pose.pose.orientation.y = 0.0;
    _dockingStation1.target_pose.pose.orientation.z = sin(angle/2);
    _dockingStation1.target_pose.pose.orientation.w = cos(angle/2);

    angle = -PI/2;
    _dockingStation2.target_pose.header.frame_id = "marrtino_map";
    _dockingStation2.target_pose.pose.position.x = -0.465;
    _dockingStation2.target_pose.pose.position.y = 0.645;
    _dockingStation2.target_pose.pose.orientation.x = 0.0;
    _dockingStation2.target_pose.pose.orientation.y = 0.0;
    _dockingStation2.target_pose.pose.orientation.z = sin(angle/2);
    _dockingStation2.target_pose.pose.orientation.w = cos(angle/2);
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

    ros::ServiceClient localizationService = n.serviceClient<std_srvs::Empty>("global_localization");

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
    ROS_INFO("Corridor end reached.");

    ROS_INFO("Rotating");
    _turn(-PI/2, -0.5, motor_control);
    ROS_INFO("Rotation complete.");

    std_srvs::Empty serviceCall;
    localizationService.call(serviceCall);
    ros::Duration(1.00).sleep();
    localizationService.call(serviceCall);
    ros::Duration(1.00).sleep();
    localizationService.call(serviceCall);
    ros::Duration(1.00).sleep();
    localizationService.call(serviceCall);
    ros::Duration(1.00).sleep();
    localizationService.call(serviceCall);
    
    /*
    ROS_INFO("Moving to arena entrance.");
    ac.sendGoal(_entrance);
    
    ros::Rate checkRate(100);
    while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
           ac.getState() == actionlib::SimpleClientGoalState::PENDING)
    {
        checkRate.sleep();
    }

    if (ac.getState() == actionlib::SimpleClientGoalState::LOST ||
        ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
        ac.cancelGoal();
        ROS_WARN("First entrance not available.");
        ROS_INFO("Going around.");

        ac.sendGoal(_alternativeEntrance);
        ac.waitForResult();
        if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR("Unable to reach target.");
            return;
        }
        ROS_INFO("Arena entrance reached.");
    }
    else
    {
        ROS_INFO("Arena entrance reached.");
        ROS_INFO("Rotating.");
        _turn(-PI/4, -0.5, motor_control);
        ROS_INFO("Rotation complete.");
    }

    ROS_INFO("Moving to docking station 1.");
    ac.sendGoal(_dockingStation1);
    ac.waitForResult();
    if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_WARN("Docking station 1 is not available.");
        ROS_INFO("Moving to docking station 2.");
        ac.sendGoal(_dockingStation2);
        ac.waitForResult();

        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR("No docking station available.");
            return;
        }
    }

    ROS_INFO("Docking station reached.");
    ROS_INFO("Ready to load.");
    */
}

void Task3::_move(float distance, double speed, ros::Publisher& motor_control)
{
    int frequency = 10;
    ros::Rate rate(frequency);
    float distancePerTick = speed/frequency;
    int ticks = 0;
    int endTicks = std::abs(static_cast<int>(distance/distancePerTick)) + 1;
    while (ticks < endTicks)
    {
        _set_velocities(speed, 0.0, motor_control);
        ros::spinOnce();
        rate.sleep();
        ++ticks;
    }
    _set_velocities(0.0, 0.0, motor_control);
}

void Task3::_turn(float angle, double speed, ros::Publisher& motor_control)
{
    int frequency = 10;
    ros::Rate rate(frequency);
    float anglePerTick = speed / frequency;
    int ticks = 0;
    int endTicks = std::abs(static_cast<int>(angle/anglePerTick)) + 1;
    while(ticks < endTicks)
    {
        _set_velocities(0.0, speed, motor_control);
        ros::spinOnce();
        rate.sleep();
        ++ticks;
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