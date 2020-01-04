#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

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
    ros::AsyncSpinner spinner(1);
    spinner.start();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok())
    {
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planninc_scene_interface;
        const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        // We can print the name of the reference frame for this robot.
        ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

        // We can also print the name of the end-effector link for this group.
        ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

        for (const auto& target : _targets)
        {
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
            joint_group_positions[0] = 1.50301;  // radians
            joint_group_positions[1] = -1.79556;
            joint_group_positions[2] = -0.849335;
            joint_group_positions[3] = -2.0492;
            joint_group_positions[4] = 1.60192;
            joint_group_positions[5] = 1.54684;
            move_group.setJointValueTarget(joint_group_positions);

            /*
            for (const auto& joint : move_group.getCurrentJointValues())
            {
                std::cout << joint << std::endl;
            }
            */
            //std::cout << move_group.getCurrentJointValues() << std::endl;
            //move_group.setPlannerId("RRTStar");
            
            moveit::planning_interface::MoveGroupInterface::Plan myPlan;
            bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Planning to reference pose failed.");
            }
            move_group.execute(myPlan);

            geometry_msgs::Pose target_pose;
            geometry_msgs::Pose transformedTargetPose;
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform("camera_rgb_optical_frame", "world", ros::Time(0));

            //target_pose.orientation.w = target.rotation.w;
            //target_pose.orientation.x = target.rotation.x;
            //target_pose.orientation.y = target.rotation.y;
            //target_pose.orientation.z = target.rotation.z;
            target_pose.position.x = target.coordinates.x;
            target_pose.position.y = target.coordinates.y;
            target_pose.position.z = target.coordinates.z;
            geometry_msgs::PoseStamped currentPose = move_group.getCurrentPose();

            tf2::doTransform(target_pose, transformedTargetPose, transformStamped);
            transformedTargetPose.position.z += 0.3;
            transformedTargetPose.position.x += 0.1;
            transformedTargetPose.orientation.w = currentPose.pose.orientation.w;
            transformedTargetPose.orientation.x = currentPose.pose.orientation.x;
            transformedTargetPose.orientation.y = currentPose.pose.orientation.y;
            transformedTargetPose.orientation.z = currentPose.pose.orientation.z;

            move_group.setPoseTarget(transformedTargetPose);
            success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Planning to reference pose failed.");
            }
            move_group.execute(myPlan);
        }
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
}