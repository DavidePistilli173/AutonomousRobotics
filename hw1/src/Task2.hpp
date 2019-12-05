#ifndef LAB_TASK2_H
#define LAB_TASK2_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2_ros/transform_listener.h>

enum class Mesh
{
    CUBE,
    HEX,
    PRISM
};

enum class Colour
{
    RED,
    GREEN,
    BLUE,
    YELLOW
};

struct DetectionObject
{
    Mesh mesh;
    Colour colour;
};

class Task2
{
    public:
        Task2();

        /* Initialise the ros node and target list. */
        bool init(int argc, char** argv);
        /* Run the target detection. */
        void run();

        /* Maximum number of objects. */
        static constexpr int N = 16;
        /* Number of object types. */
        static constexpr int TYPES = 5;
        /* Number of mesh types. */
        static constexpr int MESH_TYPES = 3;
        /* Queue length for ROS messages. */
        static constexpr int Q_LEN = 1000;
        /* Topic name. */
        static const char TOPIC_NAME_SIMULATION[];
        static const char TOPIC_NAME_REAL[];
        /* Current node name. */
        static const char NODE_NAME[];
        /* Mesh files. */
        static const std::string PATHS[MESH_TYPES];

    private:
        static pcl::PointCloud<pcl::PointXYZ>::Ptr _objects[MESH_TYPES];
        static tf2_ros::Buffer tfBuffer;
        static geometry_msgs::TransformStamped transformStamped; 

        static void _readKinectData(const sensor_msgs::PointCloud2::ConstPtr &msg);

};

#endif