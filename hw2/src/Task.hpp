#ifndef TASK_HPP
#define TASK_HPP

#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>

#include "hw1/poseArray.h"
#include "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h"

enum class Argument
{
    PATH,
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

private:
    static void _moveManipulator(const hw1::poseArray::ConstPtr &msg);

    static robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput _openGripper;
    static robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput _closeGripper;

    static std::vector<hw1::pose> _targets;
    static std::vector<shape_msgs::Mesh> _collisionMeshes;
    static std::vector<moveit_msgs::CollisionObject> _collisionObjects;
    static std::string _path;
};

#endif