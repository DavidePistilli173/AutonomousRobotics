#include "Task1.hpp"

#include <ros/ros.h>

/* Definitions of static variables. */
int Task1::targets[Task1::N];
int Task1::targetNum = 0;
std::ofstream Task1::outputFile(Task1::OF_NAME);
bool Task1::received = false;

const std::string Task1::frames[N] =
    {
        "red_cube_0",
        "red_cube_1",
        "red_cube_2",
        "red_cube_3",
        "yellow_cyl_0",
        "yellow_cyl_1",
        "green_prism_0",
        "green_prism_1",
        "green_prism_2",
        "blue_cube_0",
        "blue_cube_1",
        "blue_cube_2",
        "blue_cube_3",
        "red_prism_0",
        "red_prism_1",
        "red_prism_2"
    };


/* Definitions of methods from class Task1. */

Task1::Task1()
{}

bool Task1::init(int argc, char** argv)
{
    /* Node init. */
    ros::init(argc, argv, NODE_NAME);

    /* Argument number check. */
    if (argc > N+1)
    {
        ROS_ERROR("Too many arguments.");
        return false;
    }
    if (argc == 1)
    {
        ROS_ERROR("Too few arguments.");
        return false;
    }

    /* Link each frame_id to its id. */
    int i;
    for (i = 1; i < argc; ++i)
    {
        int id = 0;
        while (id < N && frames[id] != argv[i])
        {
            ++id;
        }
        /* If the frame_id is wrong, exit. */
        if (id == N)
        {
            ROS_ERROR("Unknown frame_id: %s", argv[i]);
            return false;
        }
        targets[i-1] = id;
    }
    targetNum = i;

    /* Check if the file is open. */        
    if (!outputFile.is_open())
    {
        ROS_ERROR("File not found.");
        return false;
    }

    return true;
}

void Task1::run()
{
    ros::NodeHandle n;
    /* Subscribe to topic TOPIC_NAME. */
    ros::Subscriber sub = n.subscribe(TOPIC_NAME, Q_LEN, printPose);
    /* Loop until a message is received. */
    while (!received)
    {
        ros::spinOnce();
    }
}

void Task1::printPose(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    /* A message has been received.*/
    received = true;

    /* Loop through all detections. */
    for (const auto &detection : msg->detections)
    {
        int j = 0;
        /* If the detection does not match a target, move on. */
        while (j < targetNum && detection.id[0] != targets[j])
        {
            ROS_INFO("Detection mismatch: detection id = %d; target id = %d;", detection.id[0], targets[j]);
            ++j;
        }

        /* If the detection matches a target, print its pose. */
        if (j < targetNum)
        {
            ROS_INFO("Object detected: %d", targets[j]);

            /* Print object frame_id. */
            outputFile << "Object: " << frames[targets[j]] << std::endl;

            /* Print object orientation. */
            outputFile  << "Orientation:" 
                        << "\n  w = " << detection.pose.pose.pose.orientation.w 
                        << "\n  x = " << detection.pose.pose.pose.orientation.x 
                        << "\n  y = " << detection.pose.pose.pose.orientation.y
                        << "\n  z = " << detection.pose.pose.pose.orientation.z 
                        << std::endl;

            /* Print object position. */
            outputFile  << "Position:" 
                        << "\n  x = " << detection.pose.pose.pose.position.x 
                        << "\n  y = " << detection.pose.pose.pose.position.y 
                        << "\n  z = " << detection.pose.pose.pose.position.z 
                        << std::endl;

            /* Print object separator. */
            outputFile << "*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*" << std::endl << std::endl;

            ROS_INFO("Data written to file (object %d)", targets[j]);
        }
    }
}