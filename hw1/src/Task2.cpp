#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types_conversion.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <sstream>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tgmath.h>

#include "hw1/poseArray.h"
#include "Task2.hpp"

/* Definitions of Task2 static variables. */
const std::string Task2::PATHS[MESH_TYPES] = 
{
    "/meshes/cube_face_2.pcd",
    "/meshes/hexagon_face.pcd",
    "/meshes/prism_base.pcd"
};

const std::string Task2::frames[TYPES] =
{
    "red_cube",
    "yellow_cylinder",
    "green_prism",
    "blue_cube",
    "red_prism"
};

const char Task2::NODE_NAME[] = "hw1_task2";
const char Task2::TOPIC_NAME_SIMULATION[] = "/camera/depth_registered/points";
const char Task2::TOPIC_NAME_REAL[] = "/camera/qhd/points";
const char Task2::DEST_TOPIC_NAME[] = "hw1_target_objects";

pcl::PointCloud<pcl::PointXYZ>::Ptr Task2::_objects[MESH_TYPES];
tf2_ros::Buffer Task2::tfBuffer;
ros::Publisher Task2::posePublisher;

double Task2::_icp_fitness_epsilon;
double Task2::_icp_transformation_epsilon;
double Task2::_icp_correspondence_distance;
int Task2::_icp_ransac_iterations;
double Task2::_icp_inlier_threshold;

double Task2::_min_x;
double Task2::_max_x;
double Task2::_min_y;
double Task2::_max_y;
double Task2::_min_z;
double Task2::_max_z;

std::vector<DetectionObject> Task2::_targets;

int Task2::_topic;

Task2::Task2(){};

bool Task2::init(int argc, char** argv)
{
    /* Node init. */
    ros::init(argc, argv, NODE_NAME);

    /* Argument number check. */
    if (argc > static_cast<int>(Argument::TOTAL))
    {
        ROS_INFO("Too many arguments.\n");
        return false;
    }
    if (argc < static_cast<int>(Argument::O2))
    {
        ROS_INFO("Too few arguments.\n");
        return false;
    }

    /* Set all target objects. */
    for (int i = static_cast<int>(Argument::O1); i < argc; ++i)
    {
        int j = 0;
        while (j < TYPES && argv[i] != frames[j]) ++j;

        /* If there is no object with the input name, exit. */
        if (j == TYPES)
        {
            ROS_ERROR("Wrong target object: %s", argv[i]);
            return false;
        }

        /* Set the appropriate object type. */
        DetectionObject target;
        switch (j)
        {
        case 0:
            target.mesh = Mesh::CUBE;
            target.colour = Colour::RED;
            break;
        case 1:
            target.mesh = Mesh::HEX;
            target.colour = Colour::YELLOW;
            break;
        case 2:
            target.mesh = Mesh::PRISM;
            target.colour = Colour::GREEN;
            break;
        case 3:
            target.mesh = Mesh::CUBE;
            target.colour = Colour::BLUE;
            break;
        case 4:
            target.mesh = Mesh::PRISM;
            target.colour = Colour::RED;
            break;
        }
        /* Add the current object to the targets. */
        _targets.push_back(target);
    }

    /* Set ICP parameters. */
    ROS_INFO("Setting ICP parameters...");
    _icp_fitness_epsilon = std::atof(argv[static_cast<int>(Argument::ICP_FITNESS_EPSILON)]);
    _icp_transformation_epsilon = std::atof(argv[static_cast<int>(Argument::ICP_TRANSFORMATION_EPSILON)]);
    _icp_correspondence_distance = std::atof(argv[static_cast<int>(Argument::ICP_MAX_CORRESPONDENCE_DISTANCE)]);
    _icp_ransac_iterations = std::atoi(argv[static_cast<int>(Argument::ICP_RANSAC_ITERATIONS)]);
    _icp_inlier_threshold = std::atof(argv[static_cast<int>(Argument::ICP_RANSAC_INLIER_THRESHOLD)]);

    /* Set point cloud filtering parameters. */
    ROS_INFO("Setting filter parameters...");
    if (std::atoi(argv[static_cast<int>(Argument::SIMULATION)]) == 0)
    {
        _topic = 0;
        _min_x = std::atof(argv[static_cast<int>(Argument::FILTER_REAL_MIN_X)]);
        _max_x = std::atof(argv[static_cast<int>(Argument::FILTER_REAL_MAX_X)]);
        _min_y = std::atof(argv[static_cast<int>(Argument::FILTER_REAL_MIN_Y)]);
        _max_y = std::atof(argv[static_cast<int>(Argument::FILTER_REAL_MAX_Y)]);
        _min_z = std::atof(argv[static_cast<int>(Argument::FILTER_REAL_MIN_Z)]);
        _max_z = std::atof(argv[static_cast<int>(Argument::FILTER_REAL_MAX_Z)]);
    }
    else
    {
        _topic = 1;
        _min_x = std::atof(argv[static_cast<int>(Argument::FILTER_SIMULATION_MIN_X)]);
        _max_x = std::atof(argv[static_cast<int>(Argument::FILTER_SIMULATION_MAX_X)]);
        _min_y = std::atof(argv[static_cast<int>(Argument::FILTER_SIMULATION_MIN_Y)]);
        _max_y = std::atof(argv[static_cast<int>(Argument::FILTER_SIMULATION_MAX_Y)]);
        _min_z = std::atof(argv[static_cast<int>(Argument::FILTER_SIMULATION_MIN_Z)]);
        _max_z = std::atof(argv[static_cast<int>(Argument::FILTER_SIMULATION_MAX_Z)]);
    }

    /* Load point clouds. */
    ROS_INFO("Loading reference point clouds...");
    for (int i = 0; i < MESH_TYPES; ++i)
    {
        _objects[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(argv[1] + PATHS[i], *_objects[i]) < 0)
        {
            ROS_ERROR("Could not load point cloud %s", PATHS[i].c_str());
            return false;
        }
    }

    return true;
}

void Task2::run()
{
    ros::NodeHandle n;
    ros::Subscriber sub;
    posePublisher = n.advertise<hw1::poseArray>(DEST_TOPIC_NAME, Q_LEN);
    ros::Rate loop_rate(10);
    
    /* Subscribe to the appropriate topic. */
    if (_topic == 0)
    {
        ROS_INFO("Subscribing to topic %s", TOPIC_NAME_REAL);
        sub = n.subscribe(TOPIC_NAME_REAL, Q_LEN, _readKinectData);
    }
    else
    {
        ROS_INFO("Subscribing to topic %s", TOPIC_NAME_SIMULATION);
        sub = n.subscribe(TOPIC_NAME_SIMULATION, Q_LEN, _readKinectData);
    }
    
    tf2_ros::TransformListener tfListener(tfBuffer);

    /* Main loop. */
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Task2::_readKinectData(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    hw1::poseArray topicOutput; // Output message.
    sensor_msgs::PointCloud2::Ptr transformedMsg(new sensor_msgs::PointCloud2);
    geometry_msgs::TransformStamped transformStamped;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msgPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    try
    {
        if (_topic == 0)
        {
            pcl::fromROSMsg(*msg, *msgPointCloud);
            pcl_ros::transformPointCloud("base_link", *msgPointCloud, *transformedPointCloud, tfBuffer);
        }
        else
        {
            transformStamped = tfBuffer.lookupTransform("camera_ir_optical_frame", "base_link", ros::Time(0));
            tf2::doTransform(*msg, *transformedMsg, transformStamped);
            pcl::fromROSMsg(*transformedMsg, *transformedPointCloud);
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
        return;
    }
    
    

    /* Convert msg to PointCloud. */
    ROS_INFO("Point cloud received.");
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    /* Transform point cloud coordinates from camera_link to base_link. */
    ROS_INFO("Applying transformations.");

    /* Point cloud filtering. */
    ROS_INFO("Filtering point cloud.");
    pcl::PassThrough<pcl::PointXYZRGB> tableFilter;
    /* Filter z axis. */
    tableFilter.setInputCloud(transformedPointCloud);
    tableFilter.setFilterLimits(_min_z, _max_z);
    tableFilter.setFilterFieldName("z");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    tableFilter.filter(*outputCloud1);
    /* Filter y axis. */
    tableFilter.setInputCloud(outputCloud1);
    tableFilter.setFilterLimits(_min_y, _max_y);
    tableFilter.setFilterFieldName("y");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    tableFilter.filter(*outputCloud2);
    /* Filter x axis. */
    tableFilter.setInputCloud(outputCloud2);
    tableFilter.setFilterLimits(_min_x, _max_x);
    tableFilter.setFilterFieldName("x");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud3(new pcl::PointCloud<pcl::PointXYZRGB>);
    tableFilter.filter(*outputCloud3);

    /* Convert RGB to HSV. */
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBtoXYZHSV(*outputCloud3, *hsvCloud);

    /* Clustering. */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZHSV> euclideanClusters;
    euclideanClusters.setInputCloud(hsvCloud);
    euclideanClusters.setMinClusterSize(50);
    euclideanClusters.setClusterTolerance(0.065);
    euclideanClusters.extract(cluster_indices);

    std::stringstream output;
    output << "Clusters: " << cluster_indices.size() << std::endl;
    ROS_INFO("%s", output.str().c_str());    
    
    /* Store all clusters in two vectors, with and without colour information. */
    std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr> detections;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> detectionsNoHSV;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        detections.push_back(pcl::PointCloud<pcl::PointXYZHSV>::Ptr(new pcl::PointCloud<pcl::PointXYZHSV>));
        detectionsNoHSV.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            detections[j]->points.push_back(hsvCloud->points[*pit]);
            pcl::PointXYZ point;
            point.x = hsvCloud->points[*pit].x;
            point.y = hsvCloud->points[*pit].y;
            point.z = hsvCloud->points[*pit].z;
            detectionsNoHSV[j]->points.push_back(point);
        }
        ++j;
    }
    
    /* Loop through all detections. */
    for (int i = 0; i < detections.size(); ++i)
    {   
        DetectionObject detectionChoice; // Current detected object. 
        /* Compute the object's maximum height. */
        float maxHeight = 0;
        for (const auto &point : detections[i]->points)
        {
            if (point.z > maxHeight) maxHeight = point.z;
        }
        /* Compute the object's average hue. */
        float averageHue = 0.0;
        int pointCount = 0;
        for (const auto &point : detections[i]->points)
        {
            /* Filter out desaturated points (eg. AprilTags). */
            if (point.s > 0.3 && point.v > 0.3)
            { 
                averageHue += point.h;
                ++pointCount;
            }
        }
        averageHue /= pointCount;

        std::string objectName = "";
        /* Choose object type based on height and colour. */
        if (maxHeight >= 0.1)
        {
            detectionChoice = {Mesh::HEX, Colour::YELLOW};
            objectName = frames[1];
        }
        else if (maxHeight >= 0.055)
        {
            if (averageHue <= 140)
            { 
                detectionChoice = {Mesh::CUBE, Colour::RED};
                objectName = frames[0];
            }
            else
            {
                detectionChoice = {Mesh::CUBE, Colour::BLUE};
                objectName = frames[3];
            }
        }
        else
        {
            if (averageHue <= 80)
            {
                detectionChoice = {Mesh::PRISM, Colour::RED};
                objectName = frames[4];
            }
            else 
            {
                detectionChoice = {Mesh::PRISM, Colour::GREEN};
                objectName = frames[2];
            }

            /* Remove unnecessary points from the base of the prism. */
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            for (int j = 0; j < detectionsNoHSV[i]->points.size(); ++j)
            {
                if (detectionsNoHSV[i]->points[j].z < 0.8 * maxHeight)
                {
                    inliers->indices.push_back(j);
                }
            }
            extract.setInputCloud(detectionsNoHSV[i]);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*detectionsNoHSV[i]);
        }

        /* Find the target corresponding to the detected object. */
        int targetId = 0;
        while (targetId < _targets.size() && ((_targets[targetId].mesh != detectionChoice.mesh) ||
               (_targets[targetId].colour != detectionChoice.colour)))
        {
            ++targetId;
        }

        /* If the detection doesn't match a target, move to the next detection. */
        if (targetId == _targets.size())
        {
            ROS_WARN("Object not relevant.");
            continue;
        }

        /* Otherwise, find the object's pose. */
        ROS_INFO("Detection %d", i);
        ROS_INFO("Number of points: %d", detections[i]->points.size());
        ROS_INFO("Average hue: %f", averageHue);
        ROS_INFO("Max height: %f", maxHeight);

        /* Move all points to maximum height in order to match them with a 2D reference point cloud. */
        for (auto &point : detectionsNoHSV[i]->points)
        {
            point.z = maxHeight;
        }

        ROS_INFO("Matched to: %s", PATHS[static_cast<int>(detectionChoice.mesh)].c_str());

        /* ICP mathcing to find the object's pose. */
        pcl::PointCloud<pcl::PointXYZ>::Ptr lastCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(_objects[static_cast<int>(detectionChoice.mesh)]);
        icp.setInputTarget(detectionsNoHSV[i]);
        icp.setEuclideanFitnessEpsilon(_icp_fitness_epsilon);
        icp.setTransformationEpsilon(_icp_transformation_epsilon);
        icp.setMaxCorrespondenceDistance(_icp_correspondence_distance);
        icp.setRANSACIterations(_icp_ransac_iterations);
        icp.setRANSACOutlierRejectionThreshold(_icp_inlier_threshold);
        icp.align(*lastCloud);

        /* Get the object's pose. */
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        
        /* Position. */
        float x = transformation(0,3);
        float y = transformation(1,3);
        float z = transformation(2,3);

        /* Orientation. */
        tf::Matrix3x3 rotationMatrix;
        rotationMatrix.setValue(transformation(0,0), transformation(0,1), transformation(0,2),
                                transformation(1,0), transformation(1,1), transformation(1,2),
                                transformation(2,0), transformation(2,1), transformation(2,2));
        tf::Quaternion quat;
        rotationMatrix.getRotation(quat);

        /* Add the pose to the output message. */
        hw1::pose object;
        object.coordinates.x = x;
        object.coordinates.y = y;
        object.coordinates.z = z;

        object.rotation.w = quat.getW();
        object.rotation.x = quat.getX();
        object.rotation.y = quat.getY();
        object.rotation.z = quat.getZ();

        object.name = objectName;

        object.type = static_cast<int>(detectionChoice.mesh);

        topicOutput.objects.push_back(object);
    }
    /* Publish all poses. */    
    posePublisher.publish(topicOutput);
}
