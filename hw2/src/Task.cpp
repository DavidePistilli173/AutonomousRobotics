#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

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


const std::string Task::NODE_NAME = "hw2_task";
const std::string Task::POSES_TOPIC = "hw1_target_objects";
const std::string Task::PLANNING_GROUP = "manipulator";
const std::string Task::GRIPPER_TOPIC = "/left_hand/command";
robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput Task::_openGripper;
robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput Task::_closeGripper;

std::vector<hw1::pose> Task::_targets;
std::vector<shape_msgs::Mesh> Task::_collisionMeshes;
std::vector<moveit_msgs::CollisionObject> Task::_collisionObjects;
std::string Task::_path;

Task::Task()
{}

bool Task::init(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);

    _path = argv[static_cast<int>(Argument::PATH)];
    std::cout << _path << std::endl;

    //shapes::Mesh* m = shapes::createMeshFromResource("file://" + _path + "/meshes/hexagon.stl");
    shapes::Mesh* m = shapes::createMeshFromResource("package://hw2/meshes/hexagon.stl");
    shapes::ShapeMsg mesh_msg; //create a shape msg
    shapes::constructMsgFromShape(m,mesh_msg); //convert shape into a shape msg
    _collisionMeshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg)); // shape msg is assigned to mesh msg

    m = shapes::createMeshFromResource("package://hw2/meshes/triangle.stl");
    mesh_msg; //create a shape msg
    shapes::constructMsgFromShape(m,mesh_msg); //convert shape into a shape msg
    _collisionMeshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg)); // shape msg is assigned to mesh msg

    m = shapes::createMeshFromResource("package://hw2/meshes/cube.stl");
    mesh_msg; //create a shape msg
    shapes::constructMsgFromShape(m,mesh_msg); //convert shape into a shape msg
    _collisionMeshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg)); // shape msg is assigned to mesh msg

    _openGripper.rACT = 1;
    _openGripper.rGTO = 1;
    _openGripper.rSPA = 200;
    
    _closeGripper.rACT = 1;
    _closeGripper.rGTO = 1;
    _closeGripper.rPRA = 250;
    _closeGripper.rSPA = 200;
    _closeGripper.rFRA = 200;
    
    return true;
}

void Task::run()
{
    ros::NodeHandle n;
    ros::Subscriber poses = n.subscribe(POSES_TOPIC, Q_LEN, _moveManipulator);
    ros::Publisher gripper = n.advertise<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput>(GRIPPER_TOPIC, Q_LEN);
    //ros::ServiceClient gazeboGrip = n.serviceClient<intro_tutorial::srv1>("add_3_ints");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    planning_scene_interface.addCollisionObjects(_collisionObjects);

    while (ros::ok())
    {
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
            ROS_INFO("Moving to reference position.");

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
            geometry_msgs::Pose aboveObjectPose;
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform("camera_rgb_optical_frame", "world", ros::Time(0));

            target_pose.position.x = target.coordinates.x;
            target_pose.position.y = target.coordinates.y;
            target_pose.position.z = target.coordinates.z;
            geometry_msgs::PoseStamped currentPose = move_group.getCurrentPose();

            tf2::doTransform(target_pose, aboveObjectPose, transformStamped);
            aboveObjectPose.position.z += 0.3;
            aboveObjectPose.position.x += 0.1;
            aboveObjectPose.orientation.w = currentPose.pose.orientation.w;
            aboveObjectPose.orientation.x = currentPose.pose.orientation.x;
            aboveObjectPose.orientation.y = currentPose.pose.orientation.y;
            aboveObjectPose.orientation.z = currentPose.pose.orientation.z;

            move_group.setPoseTarget(aboveObjectPose);
            ROS_INFO("Moving above target object.");
            success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Planning to reference pose failed.");
            }
            move_group.execute(myPlan);

            gripper.publish(_openGripper);
            geometry_msgs::PoseStamped objectPose = move_group.getCurrentPose();
            objectPose.pose.position.z -= 0.15;
            move_group.setPoseTarget(objectPose);
            ROS_INFO("Grasping object.");
            success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Planning to reference pose failed.");
            }
            move_group.execute(myPlan);
            gripper.publish(_closeGripper);
            move_group.attachObject(target.name);

            current_state = move_group.getCurrentState();
            std::vector<double> dockingStation1;
            current_state->copyJointGroupPositions(joint_model_group, dockingStation1);
            dockingStation1[0] = 0.0958103;  // radians
            dockingStation1[1] = -1.06255;
            dockingStation1[2] = 1.20556;
            dockingStation1[3] = -1.72927;
            dockingStation1[4] = -1.53248;
            dockingStation1[5] = 1.66772;
            move_group.setJointValueTarget(dockingStation1);
            ROS_INFO("Moving to docking station 1.");
            success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Planning to reference pose failed.");
            }
            move_group.execute(myPlan);

            ros::V_string targetString;
            targetString.push_back(target.name);
            planning_scene_interface.removeCollisionObjects(targetString);
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

            _collisionObjects[i].mesh_poses[0].position.x = _targets[i].coordinates.x;
            _collisionObjects[i].mesh_poses[0].position.y = _targets[i].coordinates.y;
            _collisionObjects[i].mesh_poses[0].position.z = _targets[i].coordinates.z;
            _collisionObjects[i].mesh_poses[0].orientation.w = _targets[i].rotation.w;
            _collisionObjects[i].mesh_poses[0].orientation.x = _targets[i].rotation.x;
            _collisionObjects[i].mesh_poses[0].orientation.y = _targets[i].rotation.y;
            _collisionObjects[i].mesh_poses[0].orientation.z = _targets[i].rotation.z;
            
            _collisionObjects[i].operation = _collisionObjects[i].ADD;
            _collisionObjects[i].id = _targets[i].name;
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

            moveit_msgs::CollisionObject a;
            _collisionObjects.push_back(a);
            _collisionObjects[i].meshes.resize(1);
            if (object.name.substr(0, object.name.size()-2) == "yellow_cylinder")
            {
                _collisionObjects[i].meshes[0] = _collisionMeshes[static_cast<int>(CollisionMeshes::HEX)];
            }
            else if (object.name.substr(0, object.name.size()-2) == "green_prism" ||
                     object.name.substr(0, object.name.size()-2) == "red_prism")
            {
                _collisionObjects[i].meshes[0] = _collisionMeshes[static_cast<int>(CollisionMeshes::TRIANGLE)];
            }
            else
            {
                _collisionObjects[i].meshes[0] = _collisionMeshes[static_cast<int>(CollisionMeshes::CUBE)];
            }

            _collisionObjects[i].mesh_poses.resize(1);
            _collisionObjects[i].mesh_poses[0].position.x = _targets[i].coordinates.x;
            _collisionObjects[i].mesh_poses[0].position.y = _targets[i].coordinates.y;
            _collisionObjects[i].mesh_poses[0].position.z = _targets[i].coordinates.z;
            _collisionObjects[i].mesh_poses[0].orientation.w = _targets[i].rotation.w;
            _collisionObjects[i].mesh_poses[0].orientation.x = _targets[i].rotation.x;
            _collisionObjects[i].mesh_poses[0].orientation.y = _targets[i].rotation.y;
            _collisionObjects[i].mesh_poses[0].orientation.z = _targets[i].rotation.z;
            
            _collisionObjects[i].operation = _collisionObjects[i].ADD;
            _collisionObjects[i].id = _targets[i].name;
        }
    }
}