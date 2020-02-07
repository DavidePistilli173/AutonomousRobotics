#ifndef TASK3_HPP
#define TASK3_HPP

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
    enum class Argument
    {
        PROGRAM_NAME,
        PATH,
        SIMULATION,
        KLD_ERR,
        UPDATE_MIN_D,
        UPDATE_MIN_A,
        LASER_MIN_RANGE,
        LASER_MAX_RANGE,
        ODOM_ALPHA1,
        ODOM_ALPHA2,
        ODOM_ALPHA3,
        ODOM_ALPHA4,
        TOTAL
    };

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
    static constexpr char POSE_TOPIC[] = "amcl_pose";

private:
    move_base_msgs::MoveBaseGoal _corridorEnd;
    move_base_msgs::MoveBaseGoal _entrance;
    move_base_msgs::MoveBaseGoal _alternativeEntrance;
    move_base_msgs::MoveBaseGoal _preDockingStation;
    move_base_msgs::MoveBaseGoal _dockingStation1;
    move_base_msgs::MoveBaseGoal _dockingStation2;
    void _move(float distance, double speed, ros::Publisher& motor_control);
    void _turn(float angle, double speed, ros::Publisher& motor_control);
    void _set_velocities(float lin_vel, float ang_vel, ros::Publisher& motor_control);

    static void _getEstimatedPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    static geometry_msgs::PoseWithCovarianceStamped _currentEsimatedPose;

    /* AMCL settings. */
    double _kld_err;
    double _update_min_d;
    double _update_min_a;
    double _laser_min_range;
    double _laser_max_range;
    double _odom_alpha1;
    double _odom_alpha2;
    double _odom_alpha3;
    double _odom_alpha4;
};

#endif