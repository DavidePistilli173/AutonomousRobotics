#include "Task2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>
#include <sstream>

/* Definition of Task2 static variables. */
const std::string Task2::PATHS[MESH_TYPES] = 
{
    "/meshes/cube.pcd",
    "/meshes/hexagon.pcd",
    "/meshes/prism.pcd"
};

const char Task2::NODE_NAME[] = "hw1_task2";
const char Task2::TOPIC_NAME[] = "/camera/depth_registered/points";

Task2::Task2(){};

bool Task2::init(int argc, char** argv)
{
    /* Node init. */
    ros::init(argc, argv, NODE_NAME);

    /* Argument number check. */
    if (argc > N+2)
    {
        ROS_INFO("Too many arguments.\n");
        return false;
    }
    if (argc < 3)
    {
        ROS_INFO("Too few arguments.\n");
        return false;
    }

    /* Load point clouds. */
    for (int i = 0; i < MESH_TYPES; ++i)
    {
        _objects[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(argv[1] + PATHS[i], *_objects[i]) < 0)
        {
            ROS_ERROR("Could not load point cloud %s", PATHS[i].c_str());
            return false;
        }
    }

    /* Mesh viewer. */
    /*
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(_objects[0]);
    viewer.showCloud(_objects[1]);
    viewer.showCloud(_objects[2]);
    while (!viewer.wasStopped ())
    {
    }
    */

    return true;
}

void Task2::run()
{
    ros::NodeHandle n;
    /* Subscribe to topic TOPIC_NAME. */
    ros::Subscriber sub = n.subscribe(TOPIC_NAME, Q_LEN, _readKinectData);
    /* Loop until a message is received. */
    ros::spin();
}

void Task2::_readKinectData(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    /* Convert msg to PointCloud. */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msgPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *msgPointCloud);

    pcl::PassThrough<pcl::PointXYZRGB> tableFilter;
    
    tableFilter.setInputCloud(msgPointCloud);
    tableFilter.setFilterLimits(0.0, 1.985);
    tableFilter.setFilterFieldName("z");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    tableFilter.filter(*outputCloud1);

    tableFilter.setInputCloud(outputCloud1);
    tableFilter.setFilterLimits(-0.45, 0.3);
    tableFilter.setFilterFieldName("y");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    tableFilter.filter(*outputCloud2);

    tableFilter.setInputCloud(outputCloud2);
    tableFilter.setFilterLimits(-0.6, 0.55);
    tableFilter.setFilterFieldName("x");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud3(new pcl::PointCloud<pcl::PointXYZRGB>);
    tableFilter.filter(*outputCloud3);

    /* Downsampling????????????????????????????? */

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoRGB(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : outputCloud3->points)
    {
        pcl::PointXYZ newPoint;
        newPoint.x = point.x;
        newPoint.y = point.y;
        newPoint.z = point.z;
        cloudNoRGB->points.push_back(newPoint);
    }

    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normals;
    normals.setInputCloud(cloudNoRGB);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    normals.setSearchMethod(tree);
    normals.setRadiusSearch(0.03);
    normals.compute(*cloudNormals);

    /* RGB??? */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusters;
    euclideanClusters.setInputCloud(cloudNoRGB);
    euclideanClusters.setMinClusterSize(50);
    euclideanClusters.setClusterTolerance(0.065);
    euclideanClusters.extract(cluster_indices);

    std::stringstream output;
    output << "Vector size: " << cluster_indices.size() << std::endl;
    ROS_INFO("%s", output.str().c_str());

    int colours[] = 
    {
        100, 0, 0,
        0, 100, 0,
        0, 0, 100,
        255, 0, 0,
        0, 255, 0,
        0, 0, 255,
        255, 255, 0,
        0, 255, 255,
        255, 0, 255
    };
    int j = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr completeCloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            pcl::PointXYZRGB point;
            point.x = cloudNoRGB->points[*pit].x;
            point.y = cloudNoRGB->points[*pit].y;
            point.z = cloudNoRGB->points[*pit].z;
            point.r = colours[j];
            point.g = colours[j+1];
            point.b = colours[j+2];
            completeCloud->points.push_back (point); //*
        }
        j += 3;
    }

    
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(completeCloud);
    while (!viewer.wasStopped ())
    {
    }
    
}