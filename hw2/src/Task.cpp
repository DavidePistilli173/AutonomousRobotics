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

    /* Load command line arguments. */
    _path = argv[static_cast<int>(Argument::PATH)];
    _simulation = static_cast<bool>(std::atoi(argv[static_cast<int>(Argument::SIMULATION)]));

    _objectAttached = false; // At the beginning, the manipuator does not hold any object.

    ROS_INFO("Loading meshes.");
    /* Load cube mesh. */
    shapes::Mesh* m = shapes::createMeshFromResource("package://hw2/meshes/cube.stl");
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m,mesh_msg);
    _collisionMeshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));

    /* Load hexagon mesh. */
    m = shapes::createMeshFromResource("package://hw2/meshes/hexagon.stl");
    shapes::constructMsgFromShape(m,mesh_msg);
    _collisionMeshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));

    /* Load prism mesh. */
    m = shapes::createMeshFromResource("package://hw2/meshes/triangle.stl");
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

    /* No object has been moved yet. */
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

    ros::Duration(WAIT_TIME).sleep(); // Wait for target information.

    /* Move all targets. */
    moveit::planning_interface::MoveGroupInterface::Plan myPlan;
    bool done = false;
    while (!done)
    {
        int movedTargets = 0;
        int currentTargetIndex = 0;
        for (auto& target : _targets)
        {
            /* If there is no object attached to the manipulator, grab one. */
            if (!_objectAttached)
            {
                if (_completedTargets[target.id])
                {
                    ++movedTargets;
                    continue;
                }

                ROS_INFO("Trying to get object %d", target.id);
                /* Move to the reference position. */
                if (!_moveToReferencePosition(move_group)) return;
                ros::Duration(WAIT_TIME).sleep();

                /* Set the next goal above the target object. . */
                geometry_msgs::Pose target_pose;
                geometry_msgs::Pose aboveObjectPose;
                geometry_msgs::TransformStamped transformStamped;
                transformStamped = tfBuffer.lookupTransform("world", "camera_rgb_optical_frame", ros::Time(0));

                target_pose.position.x = target.coordinates.x;
                target_pose.position.y = target.coordinates.y;
                target_pose.position.z = target.coordinates.z;
                geometry_msgs::PoseStamped currentPose = move_group.getCurrentPose();

                tf2::doTransform(target_pose, aboveObjectPose, transformStamped);
                ROS_WARN("Pose: x=%f, y=%f, z=%f", aboveObjectPose.position.x, aboveObjectPose.position.y, aboveObjectPose.position.z);

                if (_simulation) aboveObjectPose.position.z += 0.3;
                else aboveObjectPose.position.z += 0.23;
                aboveObjectPose.position.x += 0.0075;
                aboveObjectPose.orientation.w = currentPose.pose.orientation.w;
                aboveObjectPose.orientation.x = currentPose.pose.orientation.x;
                aboveObjectPose.orientation.y = currentPose.pose.orientation.y;
                aboveObjectPose.orientation.z = currentPose.pose.orientation.z;

                /* Move above the target object. */
                move_group.setPoseTarget(aboveObjectPose);
                ROS_INFO("Moving above the target object.");
                move_group.setStartStateToCurrentState();
                bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (!success)
                {
                    ROS_WARN("Failed to move above the target object.");
                    continue;
                }
                if (!move_group.execute(myPlan))
                {
                    ROS_WARN("Failed to execute plan.");
                    continue;
                }
                ros::Duration(WAIT_TIME).sleep();

                /* Grasp the target object. */
                if (_simulation)
                {
                    gripper.publish(_openGripper);
                    /* Set the rotation angle of the gripper. */
                    double qAngle = acos(target.rotation.w);
                    double qX = target.rotation.x / sin(qAngle);
                    double qY = target.rotation.y / sin(qAngle);
                    double zAngle = 2 * atan(qY/qX);
                    ROS_WARN("Rotation target: %f", zAngle);
                    std::vector<double> targetAlignment;
                    targetAlignment.resize(NUM_MANIPULATOR_JOINTS);
                    int j = 0;
                    for (const auto& joint : move_group.getCurrentJointValues())
                    {
                        targetAlignment[j] = joint;
                        ++j;
                    }
                    ROS_WARN("Current rotation angle: %f", targetAlignment[NUM_MANIPULATOR_JOINTS - 1]);
                    targetAlignment[NUM_MANIPULATOR_JOINTS - 1] = zAngle;

                    /* Rotate the gripper. */
                    current_state = move_group.getCurrentState();
                    move_group.setJointValueTarget(targetAlignment);
                    ROS_INFO("Align with the target.");
                    move_group.setStartStateToCurrentState();
                    success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    if (!success)
                    {
                        ROS_WARN("Failed to align with the target.");
                        continue;
                    }
                    if (!move_group.execute(myPlan))
                    {
                        ROS_WARN("Failed to execute plan.");
                        continue;
                    }
                    ros::Duration(WAIT_TIME).sleep();
                }

                geometry_msgs::PoseStamped objectPose = move_group.getCurrentPose();

                /* Lower the manipulator arm depending on the object. */
                /* If the target is a prism. */
                if ((target.id >= 6 && target.id <= 8) ||
                    target.id >= 13 && target.id <= 15)
                {
                    objectPose.pose.position.z -= 0.08;
                }
                /* Otherwise. */
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
                    continue;
                }
                if (!move_group.execute(myPlan))
                {
                    ROS_WARN("Failed to execute plan.");
                    continue;
                }
                ros::Duration(WAIT_TIME).sleep();

                if (!_simulation)
                {
                    /* Approach target object */
                    if (!_approachObject(move_group, static_cast<CollisionMeshes>(target.type)))
                    {
                        ROS_WARN("Failed to plan approach.");
                        _moveToReferencePosition(move_group);
                        continue;
                    }
                    ros::Duration(WAIT_TIME).sleep();
                }

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
                        else
                        {
                            _objectAttached = true;
                        }  
                    }
                }
                else
                {
                    int a;
                    std::cout << "Go?" << std::endl;
                    std::cin >> a;
                }

                /* Return in the previous position. */
                move_group.setPoseTarget(aboveObjectPose);
                ROS_INFO("Moving above the target object.");
                move_group.setStartStateToCurrentState();
                success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (!success)
                {
                    ROS_WARN("Failed to move above the target object.");
                    continue;
                }
                if (!move_group.execute(myPlan))
                {
                    ROS_WARN("Failed to execute plan.");
                    continue;
                }
                ros::Duration(WAIT_TIME).sleep();
            }

            /* Return to the reference position. */
            if (!_moveToReferencePosition(move_group)) return;
			ros::Duration(WAIT_TIME).sleep();

			/* Moving above docking station 1. */
			current_state = move_group.getCurrentState();
            move_group.setJointValueTarget(_aboveDockingStation1);
            ROS_INFO("Moving above docking station 1.");
			move_group.setStartStateToCurrentState();
            bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Failed to move above docking station 1.");
				continue;
            }
            if (!move_group.execute(myPlan))
			{
				ROS_WARN("Failed to execute plan.");
				continue;
			}
			ros::Duration(WAIT_TIME).sleep();

            /* Move to docking station 1. */
            if (!_simulation)
            {
                current_state = move_group.getCurrentState();
                move_group.setJointValueTarget(_dockingStation1);
                ROS_INFO("Moving to docking station 1.");
                move_group.setStartStateToCurrentState();
                success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (!success)
                {
                    ROS_WARN("Failed to move to docking station 1.");
                    continue;
                }
                if (!move_group.execute(myPlan))
                {
                    ROS_WARN("Failed to execute plan.");
                    continue;
                }
                ros::Duration(WAIT_TIME).sleep();
            }

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
                    _objectAttached = false;
                }
            }
            else
            {
                int a;
                std::cout << "Go?" << std::endl;
                std::cin >> a;
            }

			/* Moving above docking station 1. */
            if (!_simulation)
            {
                current_state = move_group.getCurrentState();
                move_group.setJointValueTarget(_aboveDockingStation1);
                ROS_INFO("Moving above docking station 1.");
                move_group.setStartStateToCurrentState();
                success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (!success)
                {
                    ROS_WARN("Failed to move above docking station 1.");
                    continue;
                }
                if (!move_group.execute(myPlan))
                {
                    ROS_WARN("Failed to execute plan.");
                    continue;
                }
                ros::Duration(WAIT_TIME).sleep();
            }
            ++currentTargetIndex;
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
            _collisionObjects[i].mesh_poses[0].position.x = _targets[i].coordinates.x;
            _collisionObjects[i].mesh_poses[0].position.y = _targets[i].coordinates.y;
            _collisionObjects[i].mesh_poses[0].position.z = _targets[i].coordinates.z;

            double qAngle = acos(object.rotation.w);
            double qX = object.rotation.x / sin(qAngle);
            double qY = object.rotation.y / sin(qAngle);
            double zAngle = atan(qY/qX);

            _targets[i].rotation.w = cos(zAngle);
            _targets[i].rotation.x = 0.0;
            _targets[i].rotation.y = 0.0;
            _targets[i].rotation.z = sin(zAngle);

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
            newObject.type = object.type;

            newObject.coordinates.x = object.coordinates.x;
            newObject.coordinates.y = object.coordinates.y;
            newObject.coordinates.z = object.coordinates.z;

            double qAngle = acos(object.rotation.w);
            double qX = object.rotation.x / sin(qAngle);
            double qY = object.rotation.y / sin(qAngle);
            double zAngle = atan(qY/qX);

            newObject.rotation.w = cos(zAngle);
            newObject.rotation.x = 0.0;
            newObject.rotation.y = 0.0;
            newObject.rotation.z = sin(zAngle);

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
                _collisionObjects[i].meshes[0] = _collisionMeshes[static_cast<int>(CollisionMeshes::PRISM)];
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
    move_group.setStartStateToCurrentState();
	move_group.setJointValueTarget(_referencePosition);
    ROS_INFO("Moving to reference position.");
    moveit::planning_interface::MoveGroupInterface::Plan myPlan;
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

bool Task::_approachObject(moveit::planning_interface::MoveGroupInterface& move_group, CollisionMeshes targetType)
{
    /*
    ROS_WARN("Approach? 0 to end.");
    int a = 1;
    bool done = false;
    while (!done)
    {
        std::cin >> a;
        if (a == 0) done = true;
        else
        {
            geometry_msgs::PoseStamped targetPose = move_group.getCurrentPose();
            targetPose.pose.position.z -= 0.01;
            ROS_WARN("Z: %f", targetPose.pose.position.z);
            move_group.setPoseTarget(targetPose);
            ROS_INFO("Approaching object.");
            moveit::planning_interface::MoveGroupInterface::Plan myPlan;
            bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Planning failed.");
                return false;
            }
            if (!move_group.execute(myPlan))
            {
                ROS_WARN("Failed to approach object.");
                return false;
            }
        }
    }
    return true;
    */
    
    geometry_msgs::PoseStamped currentPose = move_group.getCurrentPose();
    double targetZ;
    switch(targetType)
    {
    case CollisionMeshes::CUBE:
        targetZ = 1.0119 + TABLE_INCLINATION * (-currentPose.pose.position.x + TABLE_MAX_X);
        break;
    case CollisionMeshes::HEX:
        targetZ = TABLE_Z + 0.20;
        break;
    case CollisionMeshes::PRISM:
        targetZ = TABLE_Z + 0.065;
        break;
    }
    ROS_WARN("TargetZ: %f", targetZ);

    while (currentPose.pose.position.z > targetZ)
    {
        ROS_WARN("CurrentZ: %f", currentPose.pose.position.z);
        move_group.setStartStateToCurrentState();
        currentPose.pose.position.z -= APPROACH_STEP;
        move_group.setPoseTarget(currentPose);
        ROS_INFO("Approaching object.");
        moveit::planning_interface::MoveGroupInterface::Plan myPlan;
        bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
        {
            ROS_WARN("Planning failed.");
            return false;
        }
        if (!move_group.execute(myPlan))
        {
            ROS_WARN("Failed to approach object.");
            return false;
        }
    }
    return true;
    
    /*
    bool done = false;
    geometry_msgs::PoseStamped previousPose = move_group.getCurrentPose();
    geometry_msgs::PoseStamped targetPose = previousPose;
    previousPose.pose.position.z += DELTA_Z_THRESHOLD + 0.1;
    while (!done)
    {
        move_group.setStartStateToCurrentState();
        if (previousPose.pose.position.z - targetPose.pose.position.z >= DELTA_Z_THRESHOLD)
        {
            targetPose = move_group.getCurrentPose();
            previousPose = move_group.getCurrentPose();
            targetPose.pose.position.z -= 0.002;
            move_group.setPoseTarget(targetPose);
            ROS_INFO("Approaching object.");
            moveit::planning_interface::MoveGroupInterface::Plan myPlan;
            bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                ROS_WARN("Planning failed.");
                return false;
            }
            done = !static_cast<bool>(move_group.execute(myPlan));
        }
        else
        {
            done = true;
        }
        ros::Duration(1.0).sleep();
    }
	return true;*/
}