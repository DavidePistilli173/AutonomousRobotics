#include <ros/ros.h>

#include "Task.hpp"

std::string NODE_NAME = "hw2_task";
std::string PLANNING_GROUP = "manipulator";

int main(int argc, char** argv)
{
    Task task;
    if (!task.init(argc, argv))
    {
        ROS_ERROR("Unable to intialise program.");
        return -1;
    }
    task.run();
    return 0;
}
