#include "Task1.hpp"

#include <ros/ros.h>

/* Static variables definition. */
int Task1::targets[Task1::N];
int Task1::targetNum = 0;

Task1::Task1()
{}

bool Task1::init(int argc, char** argv)
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
            ROS_INFO("Unknown frame_id\n");
            return false;
        }
        targets[i-1] = id;
    }
    targetNum = i;
    return true;
}

void Task1::run() const
{
    ros::NodeHandle n;
    /* Subscribe to topic tag_detections. */
    ros::Subscriber sub = n.subscribe(TOPIC_NAME, Q_LEN, printPose);
    ros::spin();
}

/* Requires as parameter a ConstPtr of the appropriate type. */
void Task1::printPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO("Message received: %f\n", msg->pose.pose.position.x);
}