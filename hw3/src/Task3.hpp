#ifndef TASK3_HPP
#define TASK3_HPP

#include <move_base_msgs/MoveBaseAction.h>

enum class Argument
{
    PROGRAM_NAME,
    PATH,
    SIMULATION,
    TOTAL
};

class Task3
{
public:
    Task3();

    /* Initialise the ros node and target list. */
    bool init(int argc, char** argv);
    /* Run the target detection. */
    void run();

    /* Queue length for ROS messages. */
    static constexpr int Q_LEN = 1000;
    /* Source topic name. */
    static constexpr char SOURCE_TOPIC_NAME[] = "tag_detections";
    /* Destination topic name. */
    static constexpr char DEST_TOPIC_NAME[] = "hw1_target_objects";
    /* Current node name. */
    static constexpr char NODE_NAME[] = "hw3";

private:
    move_base_msgs::MoveBaseGoal _corridorEnd;
};

#endif