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

    static constexpr double PI = 3.1415926536;
    /* Queue length for ROS messages. */
    static constexpr int Q_LEN = 1000;
    /* Source topic name. */
    static constexpr char SOURCE_TOPIC_NAME[] = "tag_detections";
    /* Destination topic name. */
    static constexpr char DEST_TOPIC_NAME[] = "hw1_target_objects";
    /* Current node name. */
    static constexpr char NODE_NAME[] = "hw3";
    static constexpr char MOTOR_TOPIC[] = "move_base/cmd_vel";

private:
    move_base_msgs::MoveBaseGoal _corridorEnd;
    move_base_msgs::MoveBaseGoal _entrance;
    move_base_msgs::MoveBaseGoal _alternativeEntrance;
    move_base_msgs::MoveBaseGoal _preDockingStation1;
    move_base_msgs::MoveBaseGoal _preDockingStation2;
    move_base_msgs::MoveBaseGoal _dockingStation1;
    move_base_msgs::MoveBaseGoal _dockingStation2;
    void _move(float distance, double speed, ros::Publisher& motor_control);
    void _turn(float angle, double speed, ros::Publisher& motor_control);
    void _set_velocities(float lin_vel, float ang_vel, ros::Publisher& motor_control);
};

#endif