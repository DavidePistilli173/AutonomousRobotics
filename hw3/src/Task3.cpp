#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include "Task3.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::PoseWithCovarianceStamped Task3::_currentEsimatedPose;

Task3::Task3()
{
    double angle = PI/2;
    _corridorEnd.target_pose.header.frame_id = "marrtino_map";
    _corridorEnd.target_pose.pose.position.x = -1.11;
    _corridorEnd.target_pose.pose.position.y = 3.10;
    _corridorEnd.target_pose.pose.orientation.x = 0.0;
    _corridorEnd.target_pose.pose.orientation.y = 0.0;
    _corridorEnd.target_pose.pose.orientation.z = sin(angle/2);
    _corridorEnd.target_pose.pose.orientation.w = cos(angle/2);

    angle = 0;
    _entrance.target_pose.header.frame_id = "marrtino_map";
    _entrance.target_pose.pose.position.x = -0.85;
    _entrance.target_pose.pose.position.y = 3.18;
    _entrance.target_pose.pose.orientation.x = 0.0;
    _entrance.target_pose.pose.orientation.y = 0.0;
    _entrance.target_pose.pose.orientation.z = sin(angle/2);
    _entrance.target_pose.pose.orientation.w = cos(angle/2);

    angle = 0;
    _alternativeEntrance.target_pose.header.frame_id = "marrtino_map";
    _alternativeEntrance.target_pose.pose.position.x = 0.5;
    _alternativeEntrance.target_pose.pose.position.y = 3.155;
    _alternativeEntrance.target_pose.pose.orientation.x = 0.0;
    _alternativeEntrance.target_pose.pose.orientation.y = 0.0;
    _alternativeEntrance.target_pose.pose.orientation.z = sin(angle/2);
    _alternativeEntrance.target_pose.pose.orientation.w = cos(angle/2);

    angle = -PI/2;
    _preDockingStation.target_pose.header.frame_id = "marrtino_map";
    _preDockingStation.target_pose.pose.position.x = -0.0;
    _preDockingStation.target_pose.pose.position.y = 1.00;
    _preDockingStation.target_pose.pose.orientation.x = 0.0;
    _preDockingStation.target_pose.pose.orientation.y = 0.0;
    _preDockingStation.target_pose.pose.orientation.z = sin(angle/2);
    _preDockingStation.target_pose.pose.orientation.w = cos(angle/2);

    angle = -PI/2;
    _dockingStation1.target_pose.header.frame_id = "marrtino_map";
    _dockingStation1.target_pose.pose.position.x = 0.17;
    _dockingStation1.target_pose.pose.position.y = 0.645;
    _dockingStation1.target_pose.pose.orientation.x = 0.0;
    _dockingStation1.target_pose.pose.orientation.y = 0.0;
    _dockingStation1.target_pose.pose.orientation.z = sin(angle/2);
    _dockingStation1.target_pose.pose.orientation.w = cos(angle/2);

    angle = -PI/2;
    _dockingStation2.target_pose.header.frame_id = "marrtino_map";
    _dockingStation2.target_pose.pose.position.x = -0.465;
    _dockingStation2.target_pose.pose.position.y = 0.645;
    _dockingStation2.target_pose.pose.orientation.x = 0.0;
    _dockingStation2.target_pose.pose.orientation.y = 0.0;
    _dockingStation2.target_pose.pose.orientation.z = sin(angle/2);
    _dockingStation2.target_pose.pose.orientation.w = cos(angle/2);
};

bool Task3::init(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);

    _kld_err = std::atof(argv[static_cast<int>(Argument::KLD_ERR)]);
    _update_min_d = std::atof(argv[static_cast<int>(Argument::UPDATE_MIN_D)]);
    _update_min_a = std::atof(argv[static_cast<int>(Argument::UPDATE_MIN_A)]);
    _laser_min_range = std::atof(argv[static_cast<int>(Argument::LASER_MIN_RANGE)]);
    _laser_max_range = std::atof(argv[static_cast<int>(Argument::LASER_MAX_RANGE)]);
    _odom_alpha1 = std::atof(argv[static_cast<int>(Argument::ODOM_ALPHA1)]);
    _odom_alpha2 = std::atof(argv[static_cast<int>(Argument::ODOM_ALPHA2)]);
    _odom_alpha3 = std::atof(argv[static_cast<int>(Argument::ODOM_ALPHA3)]);
    _odom_alpha4 = std::atof(argv[static_cast<int>(Argument::ODOM_ALPHA4)]);

    return true;
}

void Task3::run()
{
    ros::NodeHandle n;
    //_entrance.target_pose.header.stamp = ros::Time::now();

    ros::Subscriber estimated_pose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(POSE_TOPIC, Q_LEN, _getEstimatedPose);
    ros::Publisher motor_control = n.advertise<geometry_msgs::Twist>(MOTOR_TOPIC, Q_LEN);

    //ros::ServiceClient localizationService = n.serviceClient<std_srvs::Empty>("global_localization");
    ros::ServiceClient localizationConfig = n.serviceClient<dynamic_reconfigure::Reconfigure>("amcl/set_parameters");
    dynamic_reconfigure::Reconfigure amclSettings;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    dynamic_reconfigure::DoubleParameter kld_err;
    kld_err.name = "kld_err";
    kld_err.value = _kld_err;
    amclSettings.request.config.doubles.push_back(kld_err);

    dynamic_reconfigure::DoubleParameter update_min_d;
    update_min_d.name = "update_min_d";
    update_min_d.value = _update_min_d;
    amclSettings.request.config.doubles.push_back(update_min_d);

    dynamic_reconfigure::DoubleParameter update_min_a;
    update_min_a.name = "update_min_a";
    update_min_a.value = _update_min_a;
    amclSettings.request.config.doubles.push_back(update_min_a);

    dynamic_reconfigure::DoubleParameter laser_min_range;
    laser_min_range.name = "laser_min_range";
    laser_min_range.value = _laser_min_range;
    amclSettings.request.config.doubles.push_back(laser_min_range);

    dynamic_reconfigure::DoubleParameter laser_max_range;
    laser_max_range.name = "laser_max_range";
    laser_max_range.value = _laser_max_range;
    amclSettings.request.config.doubles.push_back(laser_max_range);

    dynamic_reconfigure::DoubleParameter odom_alpha1;
    odom_alpha1.name = "odom_alpha1";
    odom_alpha1.value = _odom_alpha1;
    amclSettings.request.config.doubles.push_back(odom_alpha1);

    dynamic_reconfigure::DoubleParameter odom_alpha2;
    odom_alpha2.name = "odom_alpha2";
    odom_alpha2.value = _odom_alpha2;
    amclSettings.request.config.doubles.push_back(odom_alpha2);

    dynamic_reconfigure::DoubleParameter odom_alpha3;
    odom_alpha3.name = "odom_alpha3";
    odom_alpha3.value = _odom_alpha3;
    amclSettings.request.config.doubles.push_back(odom_alpha3);

    dynamic_reconfigure::DoubleParameter odom_alpha4;
    odom_alpha4.name = "odom_alpha4";
    odom_alpha4.value = _odom_alpha4;
    amclSettings.request.config.doubles.push_back(odom_alpha4);

    localizationConfig.call(amclSettings);

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) 
        ROS_INFO("Waiting for the move_base action server"); 

    
    ROS_INFO("Moving to the entrance.");
    while (ac.sendGoalAndWait(_entrance) != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        double zAngle = 2 * acos(_currentEsimatedPose.pose.pose.orientation.z);
        if (_currentEsimatedPose.pose.pose.position.x <= -0.72 && _currentEsimatedPose.pose.pose.position.x >= -1.5 &&
            _currentEsimatedPose.pose.pose.position.y <= 3.45 && _currentEsimatedPose.pose.pose.position.y >= 2.85)
        {
            ROS_INFO("Backing off.");
            _move(-0.1, -0.1, motor_control);
        }
    }

    ROS_INFO("Corridor end reached.");

    ROS_INFO("Moving to the docking stations.");
    while (ac.sendGoalAndWait(_preDockingStation) != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        double zAngle = 2 * acos(_currentEsimatedPose.pose.pose.orientation.z);
        ROS_INFO("Current pose: x=%f, y=%f, zAngle=%f", _currentEsimatedPose.pose.pose.position.x, _currentEsimatedPose.pose.pose.position.y, zAngle);
        if (_currentEsimatedPose.pose.pose.position.x <= -0.19 && _currentEsimatedPose.pose.pose.position.x >= -0.95 &&
            _currentEsimatedPose.pose.pose.position.y <= 3.45 && _currentEsimatedPose.pose.pose.position.y >= 2.85)
        {
            ROS_INFO("Backing off.");
            _move(-0.1, -0.1, motor_control);
        }
        else if (std::abs(_preDockingStation.target_pose.pose.position.x - _currentEsimatedPose.pose.pose.position.x) <= 0.35 &&
                 std::abs(_preDockingStation.target_pose.pose.position.y - _currentEsimatedPose.pose.pose.position.y) <= 0.35)
        {
            ROS_INFO("Completing turn.");
            double deltaZ = 2 * (asin(_preDockingStation.target_pose.pose.orientation.z) - asin(_currentEsimatedPose.pose.pose.orientation.z));
            ROS_WARN("deltaZ: %f", deltaZ);
            _turn(deltaZ, -0.2, motor_control);
        }
    }

    ROS_INFO("Docking stations reached.");
}

void Task3::_move(float distance, double speed, ros::Publisher& motor_control)
{
    int frequency = 10;
    ros::Rate rate(frequency);
    float distancePerTick = speed/frequency;
    int ticks = 0;
    int endTicks = std::abs(static_cast<int>(distance/distancePerTick)) + 1;
    while (ticks < endTicks)
    {
        _set_velocities(speed, 0.0, motor_control);
        ros::spinOnce();
        rate.sleep();
        ++ticks;
    }
    _set_velocities(0.0, 0.0, motor_control);
}

void Task3::_turn(float angle, double speed, ros::Publisher& motor_control)
{
    int frequency = 10;
    ros::Rate rate(frequency);
    float anglePerTick = speed / frequency;
    int ticks = 0;
    int endTicks = std::abs(static_cast<int>(angle/anglePerTick)) + 1;
    while(ticks < endTicks)
    {
        _set_velocities(0.0, speed, motor_control);
        ros::spinOnce();
        rate.sleep();
        ++ticks;
    }
    _set_velocities(0.0, 0.0, motor_control);
}

void Task3::_set_velocities(float lin_vel, float ang_vel, ros::Publisher& motor_control)
{
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;

    motor_control.publish(msg);
}

void Task3::_getEstimatedPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    _currentEsimatedPose.pose.pose.position.x = msg->pose.pose.position.x;
    _currentEsimatedPose.pose.pose.position.y = msg->pose.pose.position.y;
    
    _currentEsimatedPose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    _currentEsimatedPose.pose.pose.orientation.x = 0.0;
    _currentEsimatedPose.pose.pose.orientation.y = 0.0;
    _currentEsimatedPose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
}