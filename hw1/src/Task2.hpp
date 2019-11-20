#ifndef LAB_TASK2_H
#define LAB_TASK2_H

#include <pcl/PCLPointCloud2.h>
#include <string>

struct HW_PointCloud
{
    std::string name;
    pcl::PCLPointCloud2 pc; 
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
        static constexpr char TOPIC_NAME[] = "/camera/depth_registered/points";
        /* Current node name. */
        static constexpr char NODE_NAME[] = "hw1_task2";
        /* Mesh files. */
        static constexpr char CUBE_PATH[] = "/home/cjm036653/Robotics-WS/src/Lab/hw1/meshes/cube.obj";

    private:
        HW_PointCloud _objects[MESH_TYPES];
};

#endif