#ifndef TASK_HPP
#define TASK_HPP

#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>

#include "gazebo_ros_link_attacher/Attach.h"
#include "hw1/poseArray.h"
#include "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h"

enum class Argument
{
    PROGRAM_NAME,
    PATH,
    SIMULATION,
    TOTAL
};

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

    static const std::string NODE_NAME;
    static const std::string POSES_TOPIC;
    static const std::string PLANNING_GROUP;
    static const std::string GRIPPER_TOPIC;

    static const int Q_LEN = 1000;
    static const int NUM_TARGETS = 16;
    static const int NUM_MANIPULATOR_JOINTS = 6;

private:
    static void _updateTargets(const hw1::poseArray::ConstPtr &msg);
    static std::string _getModelName(const int objectId);

	bool _moveToReferencePosition(moveit::planning_interface::MoveGroupInterface& move_group);

    static robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput _openGripper;
    static robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput _closeGripper;
    static gazebo_ros_link_attacher::Attach _gazeboPluginRequest;

    static std::vector<hw1::pose> _targets;
    static bool _completedTargets[NUM_TARGETS];
    static std::vector<shape_msgs::Mesh> _collisionMeshes;
    static std::vector<moveit_msgs::CollisionObject> _collisionObjects;
    static std::vector<double> _referencePosition;
	static std::vector<double> _aboveDockingStation1;
    static std::vector<double> _dockingStation1;
    static std::string _path;

    bool _objectAttached;

    static bool _simulation;
};

#endif
