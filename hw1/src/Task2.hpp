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
        /* Queue length for ROS messages. */
        static constexpr int Q_LEN = 1000;
        /* Topic name. */
        static constexpr char TOPIC_NAME[] = "/camera/depth_registered/points";
        /* Current node name. */
        static constexpr char NODE_NAME[] = "hw1_task2";

        /* Object ids. */
        //static const std::string frames[N];
};