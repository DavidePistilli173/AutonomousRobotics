#ifndef LAB_TASK1_H
#define LAB_TASK1_H

#include <fstream>
#include <string>
#include "apriltag_ros/AprilTagDetectionArray.h"

class Task1
{
    public:
        Task1();

        /* Initialise the ros node and target list. */
        bool init(int argc, char** argv);
        /* Run the target detection. */
        void run();

        /* Maximum number of objects. */
        static constexpr int N = 16;
        /* Queue length for ROS messages. */
        static constexpr int Q_LEN = 1000;
        /* Topic name. */
        static constexpr char TOPIC_NAME[] = "tag_detections";
        /* Current node name. */
        static constexpr char NODE_NAME[] = "hw1_task1";
        /* Output file name. */
        static constexpr char OF_NAME[] = "poses.txt";

        /* Frame ids. */
        static const std::string frames[N];

    private:
        /* Callback for subscription to topic TOPIC_NAME. */
        static void printPose(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        /* Objects for which we want to print the pose. */
        static int targets[N];
        /* Number of targets. */
        static int targetNum;
        /* Output file stream. */
        static std::ofstream outputFile;
};

#endif