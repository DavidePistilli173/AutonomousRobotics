#ifndef LAB_TASK1_H
#define LAB_TASK1_H

#include <fstream>
#include <string>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <ros/ros.h>

#include "lab.hpp"

class Task1
{
public:
    Task1();

    /* Initialise the ros node and target list. */
    bool init(int argc, char** argv);
    /* Run the target detection. */
    void run();

    /* Number of arguments not related to objects. */
    static constexpr int ARGC_OFFSET = 2;
    /* Source topic name. */
    static constexpr char SOURCE_TOPIC_NAME[] = "tag_detections";
    /* Destination topic name. */
    static constexpr char DEST_TOPIC_NAME[] = "hw1_target_objects";
    /* Current node name. */
    static constexpr char NODE_NAME[] = "hw1_task1";

    /* Frame ids. */
    static const std::string frames[lab::N];
    /* Output file name. */
    static const std::string OF_NAME;

private:
    /* Callback for subscription to topic TOPIC_NAME. */
    static void _printPose(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

    static ros::Publisher posePublisher;

    /* Objects for which we want to print the pose. */
    static int _targets[lab::N];
    /* Number of targets. */
    static int _targetNum;
    /* Output file stream. */
    static std::ofstream _outputFile;
    /* Message status. */
    static bool _received;
};

#endif