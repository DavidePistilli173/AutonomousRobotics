#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include "Task.hpp"


const std::string Task::NODE_NAME = "hw2";
const std::string Task::POSES_TOPIC = "hw1_target_objects";
const std::string Task::PLANNING_GROUP = "manipulator";
std::vector<hw1::pose> Task::_targets;

Task::Task()
{}

bool Task::init(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME); 
    return true;
}

void Task::run()
{
    ros::NodeHandle n;
    ros::Subscriber poses = n.subscribe(POSES_TOPIC, Q_LEN, _moveManipulator);
    
    while (ros::ok())
    {
        ros::spinOnce();
    }
}

void Task::_moveManipulator(const hw1::poseArray::ConstPtr &msg)
{
    /* Update object poses. */
    ROS_INFO("Updating object poses.");
    for (const auto &object : msg->objects)
    {
        int i = 0;
        while (i < _targets.size() && _targets[i].name != object.name) ++i;

        if (i < _targets.size())
        {
            ROS_INFO("Updating target: %s", object.name.c_str());
            _targets[i].coordinates.x = object.coordinates.x;
            _targets[i].coordinates.y = object.coordinates.y;
            _targets[i].coordinates.z = object.coordinates.z;

            _targets[i].rotation.w = object.rotation.w;
            _targets[i].rotation.x = object.rotation.x;
            _targets[i].rotation.y = object.rotation.y;
            _targets[i].rotation.z = object.rotation.z;
        }
        else
        {
            ROS_INFO("Adding new target: %s", object.name.c_str());
            hw1::pose newObject;

            newObject.name = object.name;

            newObject.coordinates.x = object.coordinates.x;
            newObject.coordinates.y = object.coordinates.y;
            newObject.coordinates.z = object.coordinates.z;

            newObject.rotation.w = object.rotation.w;
            newObject.rotation.x = object.rotation.x;
            newObject.rotation.y = object.rotation.y;
            newObject.rotation.z = object.rotation.z;

            _targets.push_back(newObject);
        }
    }

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planninc_scene_interface;
    const robot_state::JointModelGroup* _joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());


    geometry_msgs::Pose reference_pose;
    reference_pose.orientation.w = 1.0;
    //reference_pose.position.x = 
}