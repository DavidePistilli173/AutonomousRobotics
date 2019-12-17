#include "Task1.hpp"

#include "hw1/poseArray.h"

/* Definitions of static variables. */
int Task1::_targets[Task1::N];
int Task1::_targetNum = 0;
std::ofstream Task1::_outputFile;
bool Task1::_received = false;

ros::Publisher Task1::posePublisher;

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
const std::string Task1::OF_NAME = "/poses.txt";


/* Definitions of methods from class Task1. */

Task1::Task1()
{}

bool Task1::init(int argc, char** argv)
{
    /* Node init. */
    ros::init(argc, argv, NODE_NAME);

    /* Argument number check. */
    if (argc > N+ARGC_OFFSET)
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
    for (i = ARGC_OFFSET; i < argc; ++i)
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
        _targets[i-1] = id;
    }
    _targetNum = i;

    std::string fileName = argv[1] + OF_NAME;
    _outputFile.open(fileName);
    /* Check if the file is open. */        
    if (!_outputFile.is_open())
    {
        ROS_ERROR("Error while opening output file %s.", fileName.c_str());
        return false;
    }

    return true;
}

void Task1::run()
{
    ros::NodeHandle n;
    /* Subscribe to topic TOPIC_NAME. */
    ros::Subscriber sub = n.subscribe(SOURCE_TOPIC_NAME, Q_LEN, _printPose);
    posePublisher = n.advertise<hw1::poseArray>(DEST_TOPIC_NAME, Q_LEN);
    ros::Rate loop_rate(10);
    /* Loop until a message is received. */
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Task1::_printPose(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    /* A message has been received.*/
    _received = true;
    hw1::poseArray topicOutput;

    /* Loop through all detections. */
    for (const auto &detection : msg->detections)
    {
        int j = 0;
        /* If the detection does not match a target, move on. */
        while (j < _targetNum && detection.id[0] != _targets[j])
        {
            ROS_INFO("Detection mismatch: detection id = %d; target id = %d;", detection.id[0], _targets[j]);
            ++j;
        }

        /* If the detection matches a target, print its pose. */
        if (j < _targetNum)
        {
            ROS_INFO("Object detected: %d", _targets[j]);

            if (!_received)
            {
                /* Print object frame_id. */
                _outputFile << "Object: " << frames[_targets[j]] << std::endl;

                /* Print object orientation. */
                _outputFile  << "Orientation:" 
                            << "\n  w = " << detection.pose.pose.pose.orientation.w 
                            << "\n  x = " << detection.pose.pose.pose.orientation.x 
                            << "\n  y = " << detection.pose.pose.pose.orientation.y
                            << "\n  z = " << detection.pose.pose.pose.orientation.z 
                            << std::endl;

                /* Print object position. */
                _outputFile  << "Position:" 
                            << "\n  x = " << detection.pose.pose.pose.position.x 
                            << "\n  y = " << detection.pose.pose.pose.position.y 
                            << "\n  z = " << detection.pose.pose.pose.position.z 
                            << std::endl;

                /* Print object separator. */
                _outputFile << "*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*" << std::endl << std::endl;

                ROS_INFO("Data written to file (object %d)", _targets[j]);
            }

            /* Publish the object pose to be used in later nodes. */
            hw1::pose objPose;

            objPose.rotation.w = detection.pose.pose.pose.orientation.w;
            objPose.rotation.x = detection.pose.pose.pose.orientation.x;
            objPose.rotation.y = detection.pose.pose.pose.orientation.y;
            objPose.rotation.z = detection.pose.pose.pose.orientation.z;

            objPose.coordinates.x = detection.pose.pose.pose.position.x;
            objPose.coordinates.y = detection.pose.pose.pose.position.y;
            objPose.coordinates.z = detection.pose.pose.pose.position.z;

            objPose.name = frames[j];   

            topicOutput.objects.push_back(objPose);
        }
    }

    posePublisher.publish(topicOutput);
}