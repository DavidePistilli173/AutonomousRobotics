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

/*
for (const auto& joint : move_group.getCurrentJointValues())
{
    std::cout << joint << std::endl;
}
*/

const std::string Task::NODE_NAME = "hw2_task";
const std::string Task::POSES_TOPIC = "hw1_target_objects";
const std::string Task::PLANNING_GROUP = "manipulator";
const std::string Task::GRIPPER_TOPIC = "/left_hand/command";
robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput Task::_openGripper;
robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput Task::_closeGripper;
gazebo_ros_link_attacher::Attach Task::_gazeboPluginRequest;

std::vector<hw1::pose> Task::_targets;
bool Task::_completedTargets[Task::NUM_TARGETS];
std::vector<shape_msgs::Mesh> Task::_collisionMeshes;
std::vector<moveit_msgs::CollisionObject> Task::_collisionObjects;
std::string Task::_path;
std::vector<double> Task::_referencePosition;
std::vector<double> Task::_aboveDockingStation1;
std::vector<double> Task::_dockingStation1;

bool Task::_simulation;

Task::Task()
{}

bool Task::init(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);

    _path = argv[static_cast<int>(Argument::PATH)];
    _simulation = static_cast<bool>(std::atoi(argv[static_cast<int>(Argument::SIMULATION)]));

    ROS_INFO("Loading meshes.");
    /* Load hexagon mesh. */
    shapes::Mesh* m = shapes::createMeshFromResource("package://hw2/meshes/hexagon.stl");
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m,mesh_msg);
    _collisionMeshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));

    /* Load prism mesh. */
    m = shapes::createMeshFromResource("package://hw2/meshes/triangle.stl");
    shapes::constructMsgFromShape(m,mesh_msg);
    _collisionMeshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));

    /* Load cube mesh. */
    m = shapes::createMeshFromResource("package://hw2/meshes/cube.stl");
    shapes::constructMsgFromShape(m,mesh_msg);
    _collisionMeshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));

    /* Parameters required to open the gripper. */
    _openGripper.rACT = 1;
    _openGripper.rGTO = 1;
    _openGripper.rSPA = 200;
    
    /* Parameters required to close the gripper. */
    _closeGripper.rACT = 1;
    _closeGripper.rGTO = 1;
    _closeGripper.rPRA = 250;
    _closeGripper.rSPA = 200;
    _closeGripper.rFRA = 200;

    _gazeboPluginRequest.request.model_name_1 = "ar_ur5";
    _gazeboPluginRequest.request.link_name_1 = "wrist_3_link";

    if (_simulation)
    {
        /* Set the reference position. */
        _referencePosition.resize(NUM_MANIPULATOR_JOINTS);
        _referencePosition[0] = 1.50301;
        _referencePosition[1] = -1.79556;
        _referencePosition[2] = -0.849335;
        _referencePosition[3] = -2.0492;
        _referencePosition[4] = 1.60192;
        _referencePosition[5] = 1.54684;

        /* Set the position of the docking station 1. */
        _dockingStation1.resize(NUM_MANIPULATOR_JOINTS);
        _dockingStation1[0] = 0.0958103;
        _dockingStation1[1] = -1.06255;
        _dockingStation1[2] = 1.20556;
        _dockingStation1[3] = -1.72927;
        _dockingStation1[4] = -1.53248;
        _dockingStation1[5] = 1.66772;
    }
    else
    {
        /* Set the reference position. */
        _referencePosition.resize(NUM_MANIPULATOR_JOINTS);
        _referencePosition[0] = -1.61; // Base joint
        _referencePosition[1] = -1.38;
        _referencePosition[2] = 1.24;
        _referencePosition[3] = -1.44;
        _referencePosition[4] = -1.61; // Joint for prism, 
        _referencePosition[5] = -0.02;

		/* Set the position above docking station 1. */
        _aboveDockingStation1.resize(NUM_MANIPULATOR_JOINTS);
        _aboveDockingStation1[0] = -0.02; // Base joint
        _aboveDockingStation1[1] = -1.38;
        _aboveDockingStation1[2] = 1.24;
        _aboveDockingStation1[3] = -1.44;
        _aboveDockingStation1[4] = -1.61;
        _aboveDockingStation1[5] = -0.02;

        /* Set the position of the docking station 1. */
        _dockingStation1.resize(NUM_MANIPULATOR_JOINTS);
        _dockingStation1[0] = -0.02;
        _dockingStation1[1] = -0.77;
        _dockingStation1[2] = 0.91;
        _dockingStation1[3] = -1.72;
        _dockingStation1[4] = -1.61;
        _dockingStation1[5] = -0.02;
    }

    for (auto& element : _completedTargets)
    {
        element = false;
    }
    
    return true;
}

void Task::run()
{
    ROS_INFO("Initialising topics.");
    ros::NodeHandle n;
    /* Subscribe to the topic published by Homework 1. */
    ros::Subscriber poses = n.subscribe(POSES_TOPIC, Q_LEN, _updateTargets);
    /* Publish gripper opening/closing messages. */
    ros::Publisher gripper;
    if (_simulation)
    {
        gripper = n.advertise<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput>(GRIPPER_TOPIC, Q_LEN);
    }
    /* Services related to the Gazebo plugin. */
    ros::ServiceClient gazeboAttacher;
    ros::ServiceClient gazeboDetacher;
    if (_simulation)
    {
        gazeboAttacher = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        gazeboDetacher = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    }
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Initialising tf.");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ROS_INFO("Initialising move group.");
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    ROS_INFO("Initialising planning interface.");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("Initialising joint model group.");
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    planning_scene_interface.addCollisionObjects(_collisionObjects);

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //current_state->copyJointGroupPositions(joint_model_group, _referencePosition);

    ros::Duration(2.0).sleep(); // Sleep for 1 second.
    moveit::planning_interface::MoveGroupInterface::Plan myPlan;
    bool done = false;
    while (!done)
    {
        int movedTargets = 0;
        for (auto& target : _targets)
        {
            if (_completedTargets[target.id])
            {
                ROS_WARN("Skipping object %d", target.id);
                 ++movedTargets;
                 continue;
            }

            ROS_INFO("Trying to get object %d", target.id);
            /* Move to the reference position. */
			if (!_moveToReferencePosition(move_group)) return;

			ros::Duration(1.0).sleep();

            /* Move above the target object. */
            geometry_msgs::Pose target_pose;
            geometry_msgs::Pose aboveObjectPose;
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform("world", "camera_rgb_optical_frame", ros::Time(0));

            target_pose.position.x = target.coordinates.x;
            target_pose.position.y = target.coordinates.y;
            target_pose.position.z = target.coordinates.z;
            geometry_msgs::PoseStamped currentPose = move_group.getCurrentPose();

            tf2::doTransform(target_pose, aboveObjectPose, transformStamped);
			if (_simulation)
			{
	            aboveObjectPose.position.z += 0.3;
			}
			else
			{
				aboveObjectPose.position.z += 0.23;
			}
			if (_simulation)
			{
				aboveObjectPose.position.x += 0.1;	
			}
			else
			{
				aboveObjectPose.position.x += 0.0075;
			}
            aboveObjectPose.orientation.w = currentPose.pose.orientation.w;
            aboveObjectPose.orientation.x = currentPose.pose.orientation.x;
            aboveObjectPose.orientation.y = currentPose.pose.orientation.y;
            aboveObjectPose.orientation.z = currentPose.pose.orientation.z;

            move_group.setPoseTarget(aboveObjectPose);
            ROS_INFO("Moving above target object.");
			move_group.setStartStateToCurrentState();
            bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Failed to move above target object.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
            }
            if (!move_group.execute(myPlan))
			{
				ROS_WARN("Failed to execute plan.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
			}

			ros::Duration(1.0).sleep();

            /* Grasp the target object. */
            if (_simulation)
            {
                gripper.publish(_openGripper);
            }
            geometry_msgs::PoseStamped objectPose = move_group.getCurrentPose();

            if ((target.id >= 6 && target.id <= 8) ||
                 target.id >= 13 && target.id <= 15)
            {
                objectPose.pose.position.z -= 0.11;
            }
            else
            {
                objectPose.pose.position.z -= 0.125;
            }
            move_group.setPoseTarget(objectPose);
            ROS_INFO("Grasping object.");
			move_group.setStartStateToCurrentState();
            success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Failed to move to grasping position.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
            }
            if (!move_group.execute(myPlan))
			{
				ROS_WARN("Failed to execute plan.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
			}
			ros::Duration(1.0).sleep();
            move_group.attachObject(target.name); // Attach the target collision object to the manipulator.
            /* Gripper related code. */
            if (_simulation)
            {
                gripper.publish(_closeGripper);
                std::string modelName = _getModelName(target.id);
                if (modelName == "")
                {
                    ROS_WARN("Wrong object id.");
                }
                else
                {
                    ROS_INFO("Attaching object %s", modelName.c_str());
                    _gazeboPluginRequest.request.model_name_2 = modelName + "_clone";
                    _gazeboPluginRequest.request.link_name_2 = modelName + "_link";
                    if (!gazeboAttacher.call(_gazeboPluginRequest))
                    {
                        ROS_WARN("Could not attach %s to the robot arm.", modelName.c_str());
                    }  
                }
            }
            else
            {
                // Real grasping.
            }

            /* Return to the reference position. */
            if (!_moveToReferencePosition(move_group)) return;
			ros::Duration(1.0).sleep();

			/* Moving above docking station 1. */
			current_state = move_group.getCurrentState();
            move_group.setJointValueTarget(_aboveDockingStation1);
            ROS_INFO("Moving above docking station 1.");
			move_group.setStartStateToCurrentState();
            success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Failed to move above docking station 1.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
            }
            if (!move_group.execute(myPlan))
			{
				ROS_WARN("Failed to execute plan.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
			}
			ros::Duration(1.0).sleep();

            /* Move to docking station 1. */
            current_state = move_group.getCurrentState();
            move_group.setJointValueTarget(_dockingStation1);
            ROS_INFO("Moving to docking station 1.");
			move_group.setStartStateToCurrentState();
            success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Failed to move to docking station 1.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
            }
            if (!move_group.execute(myPlan))
			{
				ROS_WARN("Failed to execute plan.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
			}
			ros::Duration(1.0).sleep();

            /* Release the target. */
            ROS_INFO("Completed movement of object %d.", target.id);
            _completedTargets[target.id] = true;
            if (_simulation)
            {
                if (!gazeboDetacher.call(_gazeboPluginRequest))
                {
                    ROS_WARN("Could not detach %s to the robot arm.", _getModelName(target.id).c_str());
                }
                else
                {
                    ros::V_string targetString;
                    targetString.push_back(target.name);
                    planning_scene_interface.removeCollisionObjects(targetString);
                    gripper.publish(_openGripper);
                }
            }
            else
            {
                // Real release.
            }

			/* Moving above docking station 1. */
			current_state = move_group.getCurrentState();
            move_group.setJointValueTarget(_aboveDockingStation1);
            ROS_INFO("Moving above docking station 1.");
			move_group.setStartStateToCurrentState();
            success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Failed to move above docking station 1.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
            }
            if (!move_group.execute(myPlan))
			{
				ROS_WARN("Failed to execute plan.");
				if (!_moveToReferencePosition(move_group)) return;
				continue;
			}
			ros::Duration(1.0).sleep();
        }

        if (movedTargets == _targets.size()) done = true;
    }

    /* Move to the reference position. */
    if (!_moveToReferencePosition(move_group)) return;
}

void Task::_updateTargets(const hw1::poseArray::ConstPtr &msg)
{
    /* Loop through all detections. */
    for (const auto &object : msg->objects)
    {
        /* Look for the current detection in the targets vector. */
        int i = 0;
        while (i < _targets.size() && _targets[i].name != object.name) ++i;

        /* If the current detection is already a target, update its pose. */
        if (i < _targets.size() && !_completedTargets[object.id])
        {
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
        }
        /* If the current detection has not already been moved, add it to the targets. */
        else
        {
            hw1::pose newObject;

            newObject.name = object.name;
            newObject.id = object.id;

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
            if (object.name.substr(0, object.name.size()-2) == "yellow_cyl")
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
            _collisionObjects[i].id = object.name;
        }
    }
}

std::string Task::_getModelName(const int objectId)
{
    switch(objectId)
    {
    case 0:
        return "cube1";
        break;
    case 1:
        return "cube2";
        break;
    case 2:
        return "cube3";
        break;
    case 3:
        return "cube4";
        break;
    case 4:
        return "Hexagon0";
        break;
    case 5:
        return "Hexagon1";
        break;
    case 6:
        return "Triangle0";
        break;
    case 7:
        return "Triangle1";
        break;
    case 8:
        return "Triangle2";
        break;
    case 9:
        return "blue_cube_1";
        break;
    case 10:
        return "blue_cube_2";
        break;
    case 11:
        return "blue_cube_3";
        break;
    case 12:
        return "blue_cube_4";
        break;
    case 13:
        return "red_triangle_1";
        break;
    case 14:
        return "red_triangle_2";
        break;
    case 15:
        return "red_triangle_3";
        break;
    default:
        return "";
        break;
    }
}

bool Task::_moveToReferencePosition(moveit::planning_interface::MoveGroupInterface& move_group)
{
	move_group.setJointValueTarget(_referencePosition);
    ROS_INFO("Moving to reference position.");
    moveit::planning_interface::MoveGroupInterface::Plan myPlan;
	move_group.setStartStateToCurrentState();
    bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
        ROS_ERROR("Planning to reference pose failed.");
		return false;
    }
    if (!move_group.execute(myPlan))
	{
		ROS_ERROR("Failed to return to reference position.");
		return false;
	}
	return true;
}
