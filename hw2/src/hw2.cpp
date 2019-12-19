#include <ros/ros.h>

#include "Task.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, Task::NODE_NAME);

    moveit::planning_interface::MoveGroupInterface move_group(Task::PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(Task::PLANNING_GROUP);

    Task task;
    if (!task.init(argc, argv))
    {
        ROS_ERROR("Unable to intialise program.");
        return -1;
    }
    task.run();
    return 0;
}