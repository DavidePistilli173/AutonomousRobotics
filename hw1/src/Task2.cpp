#include "Task2.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/io/io.h>
//#include <pcl_base.h>
//#include <pcl/PCLPointCloud2.h>
//#include <pcl/PCLPointField.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/conversions.h>
//#include <pcl/con

Task2::Task2(){};

bool Task2::init(int argc, char** argv)
{
    /* Node init. */
    ros::init(argc, argv, NODE_NAME);

    /* Argument number check. */
    if (argc > N+1)
    {
        ROS_INFO("Too many arguments.\n");
        return false;
    }
    if (argc == 1)
    {
        ROS_INFO("Too few arguments.\n");
        return false;
    }
    /*
    const std::string meshFileName = "hexagon.dae.stl";
    pcl::PolygonMesh testMesh;
    pcl::io::loadPolygonFileSTL(meshFileName, testMesh);
    */
    

    return true;
}

void Task2::run()
{
    
}