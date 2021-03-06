#ifndef LAB_TASK2_H
#define LAB_TASK2_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2_ros/transform_listener.h>

/* Mesh types. */
enum class Mesh
{
    CUBE,
    HEX,
    PRISM
};

/* Colour choices. */
enum class Colour
{
    RED,
    GREEN,
    BLUE,
    YELLOW
};

/* Command line arguments. */
enum class Argument
{
    PROGRAM_NAME, // Name of the program.
    PATH, // Home folder of the package.
    SIMULATION, // 0 when running in real environments, 1 otherwise.
    /* ICP tuning parameters. */
    ICP_FITNESS_EPSILON,
    ICP_TRANSFORMATION_EPSILON,
    ICP_MAX_CORRESPONDENCE_DISTANCE,
    ICP_RANSAC_ITERATIONS,
    ICP_RANSAC_INLIER_THRESHOLD,
    /* Input point cloud filtering parameters. */
    FILTER_SIMULATION_MIN_X,
    FILTER_SIMULATION_MAX_X,
    FILTER_SIMULATION_MIN_Y,
    FILTER_SIMULATION_MAX_Y,
    FILTER_SIMULATION_MIN_Z,
    FILTER_SIMULATION_MAX_Z,
    FILTER_REAL_MIN_X,
    FILTER_REAL_MAX_X,
    FILTER_REAL_MIN_Y,
    FILTER_REAL_MAX_Y,
    FILTER_REAL_MIN_Z,
    FILTER_REAL_MAX_Z,
    /* Possible target objects. */
    O1, O2, O3, O4, O5, O6, O7, O8,
    O9, O10, O11, O12, O13, O14, O15, O16,
    TOTAL
};

/* Type of a detected object. */
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
        /* Destination topic name. */
        static const char DEST_TOPIC_NAME[];
        /* Current node name. */
        static const char NODE_NAME[];
        /* Mesh files. */
        static const std::string PATHS[MESH_TYPES];
        /* Object names. */
        static const std::string frames[TYPES];

    private:
        static pcl::PointCloud<pcl::PointXYZ>::Ptr _objects[MESH_TYPES]; // Reference point clouds for all object types.
        static tf2_ros::Buffer tfBuffer; // Transform buffer.
        static ros::Publisher posePublisher; // Publisher used to output object poses.

        /* ICP tuning parameters. */
        static double _icp_fitness_epsilon;
        static double _icp_transformation_epsilon;
        static double _icp_correspondence_distance;
        static int _icp_ransac_iterations;
        static double _icp_inlier_threshold;

        /* Input point cloud filtering parameters. */
        static double _min_x;
        static double _max_x;
        static double _min_y;
        static double _max_y;
        static double _min_z;
        static double _max_z;

        /* Height thresholds. */
        static double _hex_height;
        static double _cube_height;

        static std::vector<DetectionObject> _targets; // Vector of all required objects.

        static int _topic; // Control variable for real/simulated environments. Values linked to Argument::SIMULATION.

        /* Input point cloud handler. */
        static void _readKinectData(const sensor_msgs::PointCloud2::ConstPtr &msg);

};

#endif