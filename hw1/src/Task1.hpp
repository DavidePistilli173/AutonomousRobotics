#ifndef LAB_TASK1_H
#define LAB_TASK1_H

#include <string>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

class Task1
{
    public:
        Task1();

        bool init(int argc, char** argv);
        void run() const;

        /* Maximum number of objects. */
        static constexpr int N = 16;
        /* Queue length for ROS messages. */
        static constexpr int Q_LEN = 1000;
        /* Topic name. */
        static constexpr char TOPIC_NAME[] = "tag_detections";
        /* Current node name. */
        static constexpr char NODE_NAME[] = "hw1_task1";

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

    private:
        /* Requires as parameter a ConstPtr of the appropriate type. */
        static void printPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

        static int targets[N];
        static int targetNum;
};

#endif