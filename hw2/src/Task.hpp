#ifndef TASK_HPP
#define TASK_HPP

#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>

#include "gazebo_ros_link_attacher/Attach.h"
#include "hw1/poseArray.h"
#include "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h"

/* List of all command line arguments. */
enum class Argument
{
    PROGRAM_NAME,
    PATH,
    SIMULATION,
    TOTAL
};

/* Types of meshes. */
enum class CollisionMeshes
{
    HEX,
    TRIANGLE,
    CUBE
};

class Task
{
public:
    Task();

    /* Initialise the ros node and target list. */
    bool init(int argc, char** argv);
    /* Run the target detection. */
    void run();

    static const std::string NODE_NAME; // Name of the current node.
    static const std::string POSES_TOPIC; // Source topic for target poses.
    static const std::string PLANNING_GROUP; // Manipulator planning group.
    static const std::string GRIPPER_TOPIC; // Topic used for simulated gripper operation.

    static const int Q_LEN = 1000;
    static const int NUM_TARGETS = 16; // Maximum number of targets.
    static const int NUM_MANIPULATOR_JOINTS = 6; // Number of manipulator joints.

private:
    static void _updateTargets(const hw1::poseArray::ConstPtr &msg); // Update target poses.
    static std::string _getModelName(const int objectId); // Return model name in gazebo.

	bool _moveToReferencePosition(moveit::planning_interface::MoveGroupInterface& move_group); // Move the manipulator to the reference position.

    static robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput _openGripper; // "Open the gripper" message.
    static robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput _closeGripper; // "Close the gripper" message.
    static gazebo_ros_link_attacher::Attach _gazeboPluginRequest; // Service used for simulated gripper operation.

    static std::vector<hw1::pose> _targets; // List of all targets.
    static bool _completedTargets[NUM_TARGETS]; // Each object has true if it has already been moved, false otherwise.
    static std::vector<shape_msgs::Mesh> _collisionMeshes; // List of all different collision meshes.
    static std::vector<moveit_msgs::CollisionObject> _collisionObjects; // List of all collision objects on the scene.
    static std::vector<double> _referencePosition; // Starting and fallback position.
	static std::vector<double> _aboveDockingStation1; // Position above the target docking station.
    static std::vector<double> _dockingStation1; // Target docking station.
    static std::string _path;

    bool _objectAttached; // True if the manipulator is carrying an object.

    static bool _simulation; // True if the simulated environment is running.
};

#endif
