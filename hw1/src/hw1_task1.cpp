#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <fstream>
#include <iostream>
#include <string>

/* Requires as parameter a ConstPtr of the appropriate type. */
void printPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO("Message received: %f\n", msg->pose.pose.position.x);
}

/* Maximum number of objects. */
constexpr int N = 16;
/* Queue length for ROS messages. */
constexpr int Q_LEN = 1000;
/* Topic name. */
constexpr char* TOPIC_NAME = "tag_detections";
/* Current node name. */
constexpr char* NODE_NAME = "hw1_task1";

/* Frame ids. */
const std::string frames[N] = 
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

int main(int argc, char** argv)
{
    /* Node init. */
    ros::init(argc, argv, NODE_NAME);

    /* Argument number check. */
    if (argc > N+1)
    {
        ROS_INFO("Too many arguments\n");
        return -1;
    }

    /* Link each frame_id to its id. */
    int targets[N];
    int i;
    for (int i = 1; i < argc; ++i)
    {
        int id = 0;
        while (id < N && frames[id] != argv[i])
        {
            ++id;
        }
        /* If the frame_id is wrong, exit. */
        if (id == N)
        {
            ROS_INFO("Unknown frame_id\n");
            return -1;
        }
        targets[i] = id;
    } 

    ros::NodeHandle n;
    /* Subscribe to topic tag_detections. */
    ros::Subscriber sub = n.subscribe(TOPIC_NAME, Q_LEN, printPose);
    ros::spin();
    return 0;
}