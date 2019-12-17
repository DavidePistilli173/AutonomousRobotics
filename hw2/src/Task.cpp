#include <
#include <ros/ros.h>

#include "Task.hpp"

std::vector<hw1::pose> Task::_targets;

bool Task::init(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
}

void Task::run()
{
    ros::NodeHandle n;
    ros::Subscriber poses = n.subscribe(POSES_TOPIC, Q_LEN, _moveManipulator);

    while (ros::ok())
    {
        ros::spinOnce();
    }
}

void Task::_moveManipulator(const hw1::poseArray::ConstPtr &msg)
{
    /* Update object poses. */
    for (const auto &object : *msg->objects)
    {
        int i = 0;
        while (i < _targets.size() && _targets[i].name != object.name) ++i;

        if (i < _targets.size())
        {
            _targets[i].coordinates.x = object.coordinates.x;
            _targets[i].coordinates.y = object.coordinates.y;
            _targets[i].coordinates.z = object.coordinates.z;

            _targets[i].rotation.w = object.rotation.w;
            _targets[i].rotation.x = object.rotation.x;
            _targets[i].rotation.y = object.rotation.y;
            _targets[i].rotation.z = object.rotation.z;
        }
        else
        {
            hw1::pose newObject;

            newObject.name = object.name;

            newObject.coordinates.x = object.coordinates.x;
            newObject.coordinates.y = object.coordinates.y;
            newObject.coordinates.z = object.coordinates.z;

            newObject.rotation.w = object.rotation.w;
            newObject.rotation.x = object.rotation.x;
            newObject.rotation.y = object.rotation.y;
            newObject.rotation.z = object.rotation.z;

            _targets.push_back(newObject);
        }
        
    }


}