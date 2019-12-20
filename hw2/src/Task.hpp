#ifndef TASK_HPP
#define TASK_HPP

#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>

#include "hw1/poseArray.h"

enum class Argument
{
    PATH,
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

    static const std::string NODE_NAME;
    static const std::string POSES_TOPIC;
    static const std::string PLANNING_GROUP;
    static const int Q_LEN = 1000;

private:
    static void _moveManipulator(const hw1::poseArray::ConstPtr &msg);

    static std::vector<hw1::pose> _targets;
};

#endif