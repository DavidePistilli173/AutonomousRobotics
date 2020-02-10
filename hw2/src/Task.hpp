#ifndef TASK_HPP
#define TASK_HPP

#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>

#include "gazebo_ros_link_attacher/Attach.h"
#include "hw1/poseArray.h"
#include "lab.hpp"
#include "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h"

/* List of all command line arguments. */
enum class Argument
{
    PROGRAM_NAME,
    PATH,
    SIMULATION,
    TOTAL
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

    static const int NUM_MANIPULATOR_JOINTS = 6; // Number of manipulator joints.
    static constexpr double WAIT_TIME = 3.0;
    //static constexpr double DELTA_Z_THRESHOLD = 0.001;
    static constexpr double TABLE_Z = 0.92;
    static constexpr double APPROACH_STEP = 0.001;
    static constexpr double TABLE_INCLINATION = 0.00926;
    static constexpr double TABLE_MIN_X = -0.58;
    static constexpr double TABLE_MAX_X = 0.5;
    static constexpr double TABLE_WIDTH = TABLE_MAX_X - TABLE_MIN_X;

private:
    static void _updateTargets(const hw1::poseArray::ConstPtr &msg); // Update target poses.
    static std::string _getModelName(const int objectId); // Return model name in gazebo.

    lab::Status _moveObject(moveit::planning_interface::MoveGroupInterface& move_group, hw1::pose target, tf2_ros::Buffer& tfBuffer,
                            ros::Publisher& gripper, ros::ServiceClient& gazeboAttacher, ros::ServiceClient& gazeboDetacher, 
                            moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
	bool _moveToReferencePosition(moveit::planning_interface::MoveGroupInterface& move_group); // Move the manipulator to the reference position.
    bool _approachObject(moveit::planning_interface::MoveGroupInterface& move_group, lab::Mesh targetType);

    static robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput _openGripper; // "Open the gripper" message.
    static robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput _closeGripper; // "Close the gripper" message.
    static gazebo_ros_link_attacher::Attach _gazeboPluginRequest; // Service used for simulated gripper operation.

    static std::vector<hw1::pose> _targets; // List of all targets.
    static bool _completedTargets[lab::N]; // Each object has true if it has already been moved, false otherwise.
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
