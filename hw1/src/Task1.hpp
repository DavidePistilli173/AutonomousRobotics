#ifndef LAB_TASK1_H
#define LAB_TASK1_H

#include <fstream>
#include <string>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <ros/ros.h>

class Task1
{
    public:
        /* Mesh types. */
        enum class Mesh
        {
            CUBE,
            HEX,
            PRISM
        };
        
        Task1();

        /* Initialise the ros node and target list. */
        bool init(int argc, char** argv);
        /* Run the target detection. */
        void run();

        /* Maximum number of objects. */
        static constexpr int N = 16;
        /* Number of arguments not related to objects. */
        static constexpr int ARGC_OFFSET = 2;
        /* Queue length for ROS messages. */
        static constexpr int Q_LEN = 1000;
        /* Source topic name. */
        static constexpr char SOURCE_TOPIC_NAME[] = "tag_detections";
        /* Destination topic name. */
        static constexpr char DEST_TOPIC_NAME[] = "hw1_target_objects";
        /* Current node name. */
        static constexpr char NODE_NAME[] = "hw1_task1";

        /* Frame ids. */
        static const std::string frames[N];
        /* Output file name. */
        static const std::string OF_NAME;

    private:
        /* Callback for subscription to topic TOPIC_NAME. */
        static void _printPose(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        static ros::Publisher posePublisher;

        /* Objects for which we want to print the pose. */
        static int _targets[N];
        /* Number of targets. */
        static int _targetNum;
        /* Output file stream. */
        static std::ofstream _outputFile;
        /* Message status. */
        static bool _received;
};

#endif