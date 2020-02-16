#ifndef TASK3_HPP
#define TASK3_HPP

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/Reconfigure.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
    static constexpr char SCAN_TOPIC[] = "scan";

    static constexpr float MAX_BACKING_OFF = 0.5;
    static constexpr float BACKING_OFF_STEP = 0.05;

private:
    move_base_msgs::MoveBaseGoal _corridorEnd;
    move_base_msgs::MoveBaseGoal _entrance;
    move_base_msgs::MoveBaseGoal _alternativeEntrance;
    move_base_msgs::MoveBaseGoal _preDockingStation;
    move_base_msgs::MoveBaseGoal _dockingStation1;
    move_base_msgs::MoveBaseGoal _dockingStation2;
    move_base_msgs::MoveBaseGoal _returnBeginning;
    move_base_msgs::MoveBaseGoal _returnCorridor;
    move_base_msgs::MoveBaseGoal _marrtinoStation;
    void _initAMCLParams(dynamic_reconfigure::ReconfigureRequest& amclSettings);
    void _initMoveBaseParams(dynamic_reconfigure::ReconfigureRequest& DWAPlannerSettings, 
                             dynamic_reconfigure::ReconfigureRequest& globalSettings,
                             dynamic_reconfigure::ReconfigureRequest& globalInflationSettings, 
                             dynamic_reconfigure::ReconfigureRequest& localSettings,
                             dynamic_reconfigure::ReconfigureRequest& localInflationSettings);
    void _moveWithPlanner(MoveBaseClient& ac, ros::Publisher& motor_control, move_base_msgs::MoveBaseGoal goal);
    void _move(float distance, double speed, ros::Publisher& motor_control);
    void _turn(float angle, double speed, ros::Publisher& motor_control);
    void _set_velocities(float lin_vel, float ang_vel, ros::Publisher& motor_control);
    bool _obstacleInFront(sensor_msgs::LaserScan scan, float coneAngle, float maxRange);
    float _obstacleBehind(sensor_msgs::LaserScan scan, float coneAngle, float maxRange);

    static void _getEstimatedPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    static void _laserScan(const sensor_msgs::LaserScan::ConstPtr &msg);
    static geometry_msgs::PoseWithCovarianceStamped _currentEsimatedPose;
    static sensor_msgs::LaserScan _scan;

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

    bool _simulation;
};

#endif