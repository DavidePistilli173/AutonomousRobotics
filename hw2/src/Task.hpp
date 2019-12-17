#ifndef TASK_HPP
#define TASK_HPP

#include <vector>

#include "hw1/poseArray.h"

class Task
{
public:
    Task();

    /* Initialise the ros node and target list. */
    bool init(int argc, char** argv);
    /* Run the target detection. */
    void run();

    static constexpr char NODE_NAME[] = "hw2";
    static constexpr char POSES_TOPIC[] = "hw1_target_objects";
    static constexpr int Q_LEN = 1000;

private:
    static void _moveManipulator(const hw1::poseArray::ConstPtr &msg);

    static std::vector<hw1::pose> _targets;
};

#endif