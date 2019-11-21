#include "Task2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>

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
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr msgPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*msgPointCloud);

    pcl::PassThrough<pcl::PointXYZ> tableFilter;
    
    tableFilter.setInputCloud(msgPointCloud);
    tableFilter.setFilterLimits(0.0, 1.985);
    tableFilter.setFilterFieldName("z");
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    tableFilter.filter(*outputCloud1);

    tableFilter.setInputCloud(outputCloud1);
    tableFilter.setFilterLimits(-0.45, 0.3);
    tableFilter.setFilterFieldName("y");
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    tableFilter.filter(*outputCloud2);

    tableFilter.setInputCloud(outputCloud2);
    tableFilter.setFilterLimits(-0.6, 0.55);
    tableFilter.setFilterFieldName("x");
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud3(new pcl::PointCloud<pcl::PointXYZ>);
    tableFilter.filter(*outputCloud3);

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(outputCloud3);
    while (!viewer.wasStopped ())
    {
    }
}