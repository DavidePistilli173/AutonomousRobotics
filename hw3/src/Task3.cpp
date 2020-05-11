#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "Task3.hpp"

geometry_msgs::PoseWithCovarianceStamped Task3::_currentEsimatedPose;
sensor_msgs::LaserScan Task3::_scan;

Task3::Task3()
{};

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

    _simulation = static_cast<bool>(std::atoi(argv[static_cast<int>(Argument::SIMULATION)]));

    double angle = 0;
    if (_simulation)
    {
        _entrance.target_pose.header.frame_id = "marrtino_map";
        _entrance.target_pose.pose.position.x = -0.85;
        _entrance.target_pose.pose.position.y = 3.18;
        _entrance.target_pose.pose.orientation.x = 0.0;
        _entrance.target_pose.pose.orientation.y = 0.0;
        _entrance.target_pose.pose.orientation.z = sin(angle/2);
        _entrance.target_pose.pose.orientation.w = cos(angle/2);
    }
    else
    {
        _entrance.target_pose.header.frame_id = "marrtino_map";
        _entrance.target_pose.pose.position.x = -0.80;
        _entrance.target_pose.pose.position.y = 3.18;
        _entrance.target_pose.pose.orientation.x = 0.0;
        _entrance.target_pose.pose.orientation.y = 0.0;
        _entrance.target_pose.pose.orientation.z = sin(angle/2);
        _entrance.target_pose.pose.orientation.w = cos(angle/2);
    }

    angle = 0;
    _alternativeEntrance.target_pose.header.frame_id = "marrtino_map";
    _alternativeEntrance.target_pose.pose.position.x = 1.0;
    _alternativeEntrance.target_pose.pose.position.y = 3.2;
    _alternativeEntrance.target_pose.pose.orientation.x = 0.0;
    _alternativeEntrance.target_pose.pose.orientation.y = 0.0;
    _alternativeEntrance.target_pose.pose.orientation.z = sin(angle/2);
    _alternativeEntrance.target_pose.pose.orientation.w = cos(angle/2);

    angle = -PI/2;
    _preDockingStation.target_pose.header.frame_id = "marrtino_map";
    _preDockingStation.target_pose.pose.position.x = 0.17;
    _preDockingStation.target_pose.pose.position.y = 1.05;
    _preDockingStation.target_pose.pose.orientation.x = 0.0;
    _preDockingStation.target_pose.pose.orientation.y = 0.0;
    _preDockingStation.target_pose.pose.orientation.z = sin(angle/2);
    _preDockingStation.target_pose.pose.orientation.w = cos(angle/2);

    angle = -PI/2;
    _dockingStation1.target_pose.header.frame_id = "marrtino_map";
    _dockingStation1.target_pose.pose.position.x = 0.17;
    _dockingStation1.target_pose.pose.position.y = 0.60;
    _dockingStation1.target_pose.pose.orientation.x = 0.0;
    _dockingStation1.target_pose.pose.orientation.y = 0.0;
    _dockingStation1.target_pose.pose.orientation.z = sin(angle/2);
    _dockingStation1.target_pose.pose.orientation.w = cos(angle/2);

    angle = -PI;
    _dockingStation2.target_pose.header.frame_id = "marrtino_map";
    _dockingStation2.target_pose.pose.position.x = -0.465;
    _dockingStation2.target_pose.pose.position.y = 0.60;
    _dockingStation2.target_pose.pose.orientation.x = 0.0;
    _dockingStation2.target_pose.pose.orientation.y = 0.0;
    _dockingStation2.target_pose.pose.orientation.z = sin(angle/2);
    _dockingStation2.target_pose.pose.orientation.w = cos(angle/2);

    angle = PI/2;
    _returnBeginning.target_pose.header.frame_id = "marrtino_map";
    _returnBeginning.target_pose.pose.position.x = 1.30;
    _returnBeginning.target_pose.pose.position.y = 2.8;
    _returnBeginning.target_pose.pose.orientation.x = 0.0;
    _returnBeginning.target_pose.pose.orientation.y = 0.0;
    _returnBeginning.target_pose.pose.orientation.z = sin(angle/2);
    _returnBeginning.target_pose.pose.orientation.w = cos(angle/2);

    
    angle = 3*PI/4;
    _preReturnCorridor.target_pose.header.frame_id = "marrtino_map";
    _preReturnCorridor.target_pose.pose.position.x = 0.75;
    _preReturnCorridor.target_pose.pose.position.y = 3.65;
    _preReturnCorridor.target_pose.pose.orientation.x = 0.0;
    _preReturnCorridor.target_pose.pose.orientation.y = 0.0;
    _preReturnCorridor.target_pose.pose.orientation.z = sin(angle/2);
    _preReturnCorridor.target_pose.pose.orientation.w = cos(angle/2);
    

    angle = -PI;
    if (_simulation)
    {
        _returnCorridor.target_pose.header.frame_id = "marrtino_map";
        _returnCorridor.target_pose.pose.position.x = -0.85;
        _returnCorridor.target_pose.pose.position.y = 3.75;
        _returnCorridor.target_pose.pose.orientation.x = 0.0;
        _returnCorridor.target_pose.pose.orientation.y = 0.0;
        _returnCorridor.target_pose.pose.orientation.z = sin(angle/2);
        _returnCorridor.target_pose.pose.orientation.w = cos(angle/2);
    }
    else
    {
        _returnCorridor.target_pose.header.frame_id = "marrtino_map";
        _returnCorridor.target_pose.pose.position.x = -0.85;
        _returnCorridor.target_pose.pose.position.y = 3.75;
        _returnCorridor.target_pose.pose.orientation.x = 0.0;
        _returnCorridor.target_pose.pose.orientation.y = 0.0;
        _returnCorridor.target_pose.pose.orientation.z = sin(angle/2);
        _returnCorridor.target_pose.pose.orientation.w = cos(angle/2);
        }

    angle = -PI/2;
    _marrtinoStation.target_pose.header.frame_id = "marrtino_map";
    _marrtinoStation.target_pose.pose.position.x = -1.2;
    _marrtinoStation.target_pose.pose.position.y = 0.075;
    _marrtinoStation.target_pose.pose.orientation.x = 0.0;
    _marrtinoStation.target_pose.pose.orientation.y = 0.0;
    _marrtinoStation.target_pose.pose.orientation.z = sin(angle/2);
    _marrtinoStation.target_pose.pose.orientation.w = cos(angle/2);    

    return true;
}

void Task3::run()
{
    ros::NodeHandle n;
    //_entrance.target_pose.header.stamp = ros::Time::now();

    ros::Subscriber laserScan = n.subscribe(SCAN_TOPIC, Q_LEN, _laserScan);
    ros::Subscriber estimated_pose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(POSE_TOPIC, Q_LEN, _getEstimatedPose);
    ros::Publisher motor_control = n.advertise<geometry_msgs::Twist>(MOTOR_TOPIC, Q_LEN);

    //ros::ServiceClient localizationService = n.serviceClient<std_srvs::Empty>("global_localization");
    ros::ServiceClient localizationConfig = n.serviceClient<dynamic_reconfigure::Reconfigure>("amcl/set_parameters");
    ros::ServiceClient DWAPlannerConfig = n.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/DWAPlannerROS/set_parameters");
    ros::ServiceClient globalConfig = n.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/global_costmap/set_parameters");
    ros::ServiceClient globalInflationConfig = n.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/global_costmap/inflation_layer/set_parameters");
    ros::ServiceClient localConfig = n.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/local_costmap/set_parameters");
    ros::ServiceClient localInflationConfig = n.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/local_costmap/inflation_layer/set_parameters");

    dynamic_reconfigure::ReconfigureRequest amclSettings;
    dynamic_reconfigure::ReconfigureResponse amclSettingsResponse;

    dynamic_reconfigure::ReconfigureRequest DWAPlannerSettings;
    dynamic_reconfigure::ReconfigureRequest globalSettings;
    dynamic_reconfigure::ReconfigureRequest globalInflationSettings;
    dynamic_reconfigure::ReconfigureRequest localSettings;
    dynamic_reconfigure::ReconfigureRequest localInflationSettings;

    dynamic_reconfigure::ReconfigureResponse DWAPlannerSettingsResponse;
    dynamic_reconfigure::ReconfigureResponse globalSettingsResponse;
    dynamic_reconfigure::ReconfigureResponse globalInflationSettingsResponse;
    dynamic_reconfigure::ReconfigureResponse localSettingsResponse;
    dynamic_reconfigure::ReconfigureResponse localInflationSettingsResponse;

    _initAMCLParams(amclSettings);
    _initMoveBaseParams(DWAPlannerSettings, globalSettings, globalInflationSettings, localSettings, localInflationSettings);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    localizationConfig.call(amclSettings, amclSettingsResponse);
    DWAPlannerConfig.call(DWAPlannerSettings, DWAPlannerSettingsResponse);
    globalConfig.call(globalSettings, globalSettingsResponse);
    globalInflationConfig.call(globalInflationSettings, globalInflationSettingsResponse);
    localConfig.call(localSettings, localSettingsResponse);
    localInflationConfig.call(localInflationSettings, localInflationSettingsResponse);

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) 
        ROS_INFO("Waiting for the move_base action server"); 

    
    ROS_INFO("Moving to the entrance.");
    _moveWithPlanner(ac, motor_control, _entrance);

    ROS_INFO("Corridor end reached.");

    bool obstacleInFront = _obstacleInFront(_scan, PI/8, 1.0);
    ROS_INFO("Object in front? %d", obstacleInFront);    
    
    ROS_INFO("Moving to the docking stations.");
    if (!obstacleInFront)
    {
        _moveWithPlanner(ac, motor_control, _alternativeEntrance);
    }

    _moveWithPlanner(ac, motor_control, _preDockingStation);

    /*
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
    */

    if (!_simulation)
    {
        if (!_obstacleInFront(_scan, PI/6, 0.9))
        {
            ROS_INFO("Moving to docking station 1");
            _moveWithPlanner(ac, motor_control, _dockingStation1);
        }
        else
        {
            ROS_INFO("Moving to docking station 2");
            _moveWithPlanner(ac, motor_control, _dockingStation2);
        }
    }
    else
    {
        if (!_obstacleInFront(_scan, PI/6, 0.8))
        {
            ROS_INFO("Moving to docking station 1");
            _moveWithPlanner(ac, motor_control, _dockingStation1);
        }
        else
        {
            ROS_INFO("Moving to docking station 2");
            _moveWithPlanner(ac, motor_control, _dockingStation2);
        }
    }
    ROS_INFO("Docking station reached.");

    ROS_INFO("Moving to the start of the return corridor.");
    _moveWithPlanner(ac, motor_control, _returnBeginning);
    ROS_INFO("Moving to the start of the return corridor (part 2).");
    _moveWithPlanner(ac, motor_control, _preReturnCorridor);
    ROS_INFO("Moving through the return corridor.");
    _moveWithPlanner(ac, motor_control, _returnCorridor);
    ROS_INFO("Going back to the start position.");
    _moveWithPlanner(ac, motor_control, _marrtinoStation);
}

void Task3::_initAMCLParams(dynamic_reconfigure::ReconfigureRequest& amclSettings)
{
    dynamic_reconfigure::DoubleParameter kld_err;
    kld_err.name = "kld_err";
    kld_err.value = _kld_err;
    amclSettings.config.doubles.push_back(kld_err);

    dynamic_reconfigure::DoubleParameter update_min_d;
    update_min_d.name = "update_min_d";
    update_min_d.value = _update_min_d;
    amclSettings.config.doubles.push_back(update_min_d);

    dynamic_reconfigure::DoubleParameter update_min_a;
    update_min_a.name = "update_min_a";
    update_min_a.value = _update_min_a;
    amclSettings.config.doubles.push_back(update_min_a);

    dynamic_reconfigure::DoubleParameter laser_min_range;
    laser_min_range.name = "laser_min_range";
    laser_min_range.value = _laser_min_range;
    amclSettings.config.doubles.push_back(laser_min_range);

    dynamic_reconfigure::DoubleParameter laser_max_range;
    laser_max_range.name = "laser_max_range";
    laser_max_range.value = _laser_max_range;
    amclSettings.config.doubles.push_back(laser_max_range);

    dynamic_reconfigure::DoubleParameter odom_alpha1;
    odom_alpha1.name = "odom_alpha1";
    odom_alpha1.value = _odom_alpha1;
    amclSettings.config.doubles.push_back(odom_alpha1);

    dynamic_reconfigure::DoubleParameter odom_alpha2;
    odom_alpha2.name = "odom_alpha2";
    odom_alpha2.value = _odom_alpha2;
    amclSettings.config.doubles.push_back(odom_alpha2);

    dynamic_reconfigure::DoubleParameter odom_alpha3;
    odom_alpha3.name = "odom_alpha3";
    odom_alpha3.value = _odom_alpha3;
    amclSettings.config.doubles.push_back(odom_alpha3);

    dynamic_reconfigure::DoubleParameter odom_alpha4;
    odom_alpha4.name = "odom_alpha4";
    odom_alpha4.value = _odom_alpha4;
    amclSettings.config.doubles.push_back(odom_alpha4);
}

void Task3::_initMoveBaseParams(dynamic_reconfigure::ReconfigureRequest& DWAPlannerSettings, 
                                dynamic_reconfigure::ReconfigureRequest& globalSettings,
                                dynamic_reconfigure::ReconfigureRequest& globalInflationSettings, 
                                dynamic_reconfigure::ReconfigureRequest& localSettings,
                                dynamic_reconfigure::ReconfigureRequest& localInflationSettings)
{
    /* Set DWA parameters. */
    dynamic_reconfigure::DoubleParameter acc_lim_x;
    acc_lim_x.name = "acc_lim_x";
    acc_lim_x.value = 0.7;
    DWAPlannerSettings.config.doubles.push_back(acc_lim_x);

    dynamic_reconfigure::DoubleParameter acc_lim_th;
    acc_lim_th.name = "acc_lim_theta";
    acc_lim_th.value = 2.0;
    DWAPlannerSettings.config.doubles.push_back(acc_lim_th);
    
    dynamic_reconfigure::DoubleParameter max_vel_x;
    max_vel_x.name = "max_vel_x";
    max_vel_x.value = 0.3;
    DWAPlannerSettings.config.doubles.push_back(max_vel_x);    
    
    dynamic_reconfigure::DoubleParameter min_vel_x;
    min_vel_x.name = "min_vel_x";
    min_vel_x.value = -0.3;
    DWAPlannerSettings.config.doubles.push_back(min_vel_x);   
    
    dynamic_reconfigure::DoubleParameter yaw_goal_tolerance;
    yaw_goal_tolerance.name = "yaw_goal_tolerance";
    yaw_goal_tolerance.value = 0.3;
    DWAPlannerSettings.config.doubles.push_back(yaw_goal_tolerance);
    
    dynamic_reconfigure::DoubleParameter xy_goal_tolerance;
    xy_goal_tolerance.name = "xy_goal_tolerance";
    xy_goal_tolerance.value = 0.2;
    DWAPlannerSettings.config.doubles.push_back(xy_goal_tolerance);
    
    dynamic_reconfigure::DoubleParameter sim_time;
    sim_time.name = "sim_time";
    sim_time.value = 3.0;
    DWAPlannerSettings.config.doubles.push_back(sim_time);         
    
    dynamic_reconfigure::DoubleParameter path_distance_bias;
    path_distance_bias.name = "path_distance_bias";
    path_distance_bias.value = 50.0;
    DWAPlannerSettings.config.doubles.push_back(path_distance_bias); 

    dynamic_reconfigure::DoubleParameter goal_distance_bias;
    goal_distance_bias.name = "goal_distance_bias";
    goal_distance_bias.value = 12.0;
    DWAPlannerSettings.config.doubles.push_back(goal_distance_bias);

    dynamic_reconfigure::DoubleParameter occdist_scale;
    occdist_scale.name = "occdist_scale";
    occdist_scale.value = 0.005;
    DWAPlannerSettings.config.doubles.push_back(occdist_scale); 
    

    /* Set common costmap parameters. */
    dynamic_reconfigure::StrParameter footprint;  
    if (_simulation)
    {
        footprint.name = "footprint";
        footprint.value = "[[-0.1275,-0.205],[-0.1275,0.205],[0.1275,0.205],[0.1275,-0.205]]";
        globalSettings.config.strs.push_back(footprint);
        localSettings.config.strs.push_back(footprint);
    }
    else
    {
        footprint.name = "footprint";
        footprint.value = "[[-0.13,-0.225],[-0.13,0.225],[0.135,0.225],[0.135,-0.225]]";
        globalSettings.config.strs.push_back(footprint);
        localSettings.config.strs.push_back(footprint);
    }

    dynamic_reconfigure::DoubleParameter resolution;
    resolution.name = "resolution";
    resolution.value = 0.01;
    globalSettings.config.doubles.push_back(resolution);
    localSettings.config.doubles.push_back(resolution);     

    dynamic_reconfigure::DoubleParameter cost_scaling_factor;
    cost_scaling_factor.name = "cost_scaling_factor";
    cost_scaling_factor.value = 10.0;
    globalInflationSettings.config.doubles.push_back(cost_scaling_factor);
    localInflationSettings.config.doubles.push_back(cost_scaling_factor); 

    /* Set inflation parameters. */
    dynamic_reconfigure::DoubleParameter global_inflation_radius;
    global_inflation_radius.name = "inflation_radius";
    global_inflation_radius.value = 0.07;
    globalInflationSettings.config.doubles.push_back(global_inflation_radius);

    dynamic_reconfigure::DoubleParameter local_inflation_radius;
    local_inflation_radius.name = "inflation_radius";
    local_inflation_radius.value = 0.01;
    localInflationSettings.config.doubles.push_back(local_inflation_radius);
}

void Task3::_moveWithPlanner(MoveBaseClient& ac, ros::Publisher& motor_control, move_base_msgs::MoveBaseGoal goal)
{
    int attempt = 1;
    while (ac.sendGoalAndWait(goal) != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Backing off no.%d", attempt);
        float backObstacleRange = _obstacleBehind(_scan, PI/2.75, 2*MAX_BACKING_OFF);
        float backingOffDistance = attempt*BACKING_OFF_STEP;
        ROS_WARN("backObstacleRange = %f, backingOffDistance = %f", backObstacleRange, backingOffDistance);
        if (backingOffDistance > MAX_BACKING_OFF) backingOffDistance = BACKING_OFF_STEP;
        if (backObstacleRange == -1 || backingOffDistance < backObstacleRange - 0.35)
        {
            _move(-backingOffDistance, -0.1, motor_control);
        }
        else
        {
            _move(-backObstacleRange/2, -0.1, motor_control);
        }
        ++attempt;
    }
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

void Task3::_laserScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    sensor_msgs::LaserScan scan;

    //Angle of first beam (ranges[0]) (-PI)
    _scan.angle_min = msg->angle_min;

    //Angle of last beam (ranges[400]) (PI)
    _scan.angle_max = msg->angle_max;
    
    //Step between beam angles (0.015747 rad = PI/199.50 (?))
    _scan.angle_increment = msg->angle_increment;

    //Laser scan ranges in m: angle(laser[i + 1]) = angle(laser[i]) + angle_increment
    _scan.ranges = msg->ranges;

    //Max laser range (to remove erroneous scans)
    _scan.range_max = msg->range_max;
}

//coneAngle: Angle in which to consider scans (starting from marrtino front)
bool Task3::_obstacleInFront(sensor_msgs::LaserScan scan, float coneAngle, float maxRange)
{
    //Necessary angle increments to span desired angle
    int steps = coneAngle / scan.angle_increment;
    //Array containing only scans in the selected cone
    float coneScans[steps];

    //Number of points to be detected before an obstacle is identified
    int obsThresh;
    if (_simulation)
    {
        obsThresh = 20;
    }
    else
    {
        obsThresh = 10;
    }
    
    //Index of marrtino front
    int frontIndex = scan.ranges.size() / 2;

    int detectedPoints = 0;
    float totalDistance = 0;
    for(int i = 0; i < steps / 2; ++i)
    {
        float range = scan.ranges[frontIndex - i];
        if(range < maxRange && range > 0)
        {
            coneScans[i] = scan.ranges[frontIndex - i];
            ++detectedPoints;
            totalDistance += range;
        }
        else
            coneScans[i] = 0;
        
        range = scan.ranges[frontIndex + i];
        if(range < maxRange && range > 0)
        {
            coneScans[2 * i] = scan.ranges[frontIndex + i];
            ++detectedPoints;
            totalDistance += range;
        }
        else
            coneScans[2 * i] = 0;
    }

    ROS_WARN("Detected Points: %d", detectedPoints);

    if(detectedPoints >= obsThresh)
    {
        return true;
    }
    else
        return false;
}

float Task3::_obstacleBehind(sensor_msgs::LaserScan scan, float coneAngle, float maxRange)
{
    //Necessary angle increments to span desired angle
    int steps = coneAngle / scan.angle_increment;
    //Array containing only scans in the selected cone
    float coneScans[steps];

    //Number of points to be detected before an obstacle is identified
    const int obsThresh = 10;
    
    int detectedPoints = 0;
    float totalDistance = 0;
    for(int i = 0; i < steps / 2; ++i)
    {
        float range = scan.ranges[i];
        if(range < maxRange && range > 0)
        {
            coneScans[i] = scan.ranges[i];
            ++detectedPoints;
            totalDistance += range;
        }
        else
            coneScans[i] = 0;
        
        range = scan.ranges[scan.ranges.size() - i - 1];
        if(range < maxRange && range > 0)
        {
            coneScans[2 * i] = scan.ranges[scan.ranges.size() - i - 1];
            ++detectedPoints;
            totalDistance += range;
        }
        else
            coneScans[2 * i] = 0;
    }

    if(detectedPoints >= obsThresh)
    {
        return totalDistance / detectedPoints;
    }
    else
        return -1.0f;
}