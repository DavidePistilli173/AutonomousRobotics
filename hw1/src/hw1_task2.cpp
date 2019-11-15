#include "Task2.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    Task2 task;
    if (!task.init(argc, argv))
    {
        ROS_ERROR("Failed to initialise. Terminating...\n");
        return -1;
    }
    task.run();
    return 0;
}