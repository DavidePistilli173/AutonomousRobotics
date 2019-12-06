#include "Task2.hpp"

#include <tgmath.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>
#include <sstream>
#include <pcl/registration/icp.h>
#include <pcl/point_types_conversion.h>
#include <tf/transform_listener.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/registration.h>
#include <pcl_ros/transforms.h>

/* Definition of Task2 static variables. */
const std::string Task2::PATHS[MESH_TYPES] = 
{
    "/meshes/cube_face.pcd",
    "/meshes/hexagon_face.pcd",
    "/meshes/prism2.pcd"
};

const char Task2::NODE_NAME[] = "hw1_task2";
const char Task2::TOPIC_NAME_SIMULATION[] = "/camera/depth_registered/points"; //Alternative topic
const char Task2::TOPIC_NAME_REAL[] = "/camera/hd/points"; //Active topic

pcl::PointCloud<pcl::PointXYZ>::Ptr Task2::_objects[MESH_TYPES];
tf2_ros::Buffer Task2::tfBuffer;
geometry_msgs::TransformStamped Task2::transformStamped;

double Task2::_icp_fitness_epsilon;
double Task2::_icp_transformation_epsilon;
double Task2::_icp_correspondence_distance;
int Task2::_icp_ransac_iterations;
double Task2::_icp_inlier_threshold;
double Task2::_icp_translation_threshold;
double Task2::_icp_rotation_threshold;
double Task2::_icp_absolute_mse;
int Task2::_icp_max_iterations;
int Task2::_icp_max_similar_iterations;
double Task2::_icp_relative_mse;

Task2::Task2(){};

bool Task2::init(int argc, char** argv)
{
    /* Node init. */
    ros::init(argc, argv, NODE_NAME);

    /* CHECKKK */
    /* Argument number check. */
    /*
    if (argc > N+2)
    {
        ROS_INFO("Too many arguments.\n");
        return false;
    }
    if (argc < 3)
    {
        ROS_INFO("Too few arguments.\n");
        return false;
    }
    */

    _icp_fitness_epsilon = std::atof(argv[static_cast<int>(Argument::ICP_FITNESS_EPSILON)]);
    _icp_transformation_epsilon = std::atof(argv[static_cast<int>(Argument::ICP_TRANSFORMATION_EPSILON)]);
    _icp_correspondence_distance = std::atof(argv[static_cast<int>(Argument::ICP_MAX_CORRESPONDENCE_DISTANCE)]);
    _icp_ransac_iterations = std::atoi(argv[static_cast<int>(Argument::ICP_RANSAC_ITERATIONS)]);
    _icp_inlier_threshold = std::atof(argv[static_cast<int>(Argument::ICP_RANSAC_INLIER_THRESHOLD)]);
    _icp_translation_threshold = std::atof(argv[static_cast<int>(Argument::ICP_TRANSLATION_THRESHOLD)]);
    _icp_rotation_threshold = std::atof(argv[static_cast<int>(Argument::ICP_ROTATION_THRESHOLD)]);
    _icp_absolute_mse = std::atof(argv[static_cast<int>(Argument::ICP_ABSOLUTE_MSE)]);
    _icp_max_iterations = std::atoi(argv[static_cast<int>(Argument::ICP_MAX_ITERATIONS)]);
    _icp_max_similar_iterations = std::atoi(argv[static_cast<int>(Argument::ICP_MAX_SIMILAR_ITERATIONS)]);
    _icp_relative_mse = std::atof(argv[static_cast<int>(Argument::ICP_RELATIVE_MSE)]);
    /* Load point clouds. */
    for (int i = 0; i < MESH_TYPES; ++i)
    {
        _objects[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(argv[1] + PATHS[i], *_objects[i]) < 0)
        {
            ROS_ERROR("Could not load point cloud %s", PATHS[i].c_str());
            return false;
        }
    }

    /* Mesh viewer. */
    /*
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(_objects[0]);
    viewer.showCloud(_objects[1]);
    viewer.showCloud(_objects[2]);
    while (!viewer.wasStopped ())
    {
    }
    */

    return true;
}

void Task2::run()
{
    ros::NodeHandle n;
    /* Subscribe to topic TOPIC_NAME. */
    ros::Subscriber sub = n.subscribe(TOPIC_NAME_SIMULATION, Q_LEN, _readKinectData);

    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok())
    {
        /*
        try
        {
            transformStamped = tfBuffer.lookupTransform("base_link", "camera_link", ros::Time(0));
            ROS_INFO("Tf received.");
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        */

        ros::spinOnce();
    }
}

void Task2::_readKinectData(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    /* Convert msg to PointCloud. */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msgPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *msgPointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    /* Transform point cloud coordinates from camera_link to base_link. */
    //pcl_ros::transformPointCloud("base_link", *msgPointCloud, *transformedPointCloud, tfBuffer);

    /* Point cloud filtering. */
    pcl::PassThrough<pcl::PointXYZRGB> tableFilter;
    /* CAMBIARE POINT CLOUD DI INPUT CON ATTIVAZIONE TF. */
    tableFilter.setInputCloud(msgPointCloud);
    tableFilter.setFilterLimits(0.0, 1.985); //Simulation
    //tableFilter.setFilterLimits(0.0, 1.985); //Real
    tableFilter.setFilterFieldName("z");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    tableFilter.filter(*outputCloud1);

    tableFilter.setInputCloud(outputCloud1);
    tableFilter.setFilterLimits(-0.45, 0.3); //Simulation
    //tableFilter.setFilterLimits(0.0, 0.5); //Real
    tableFilter.setFilterFieldName("y");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    tableFilter.filter(*outputCloud2);

    tableFilter.setInputCloud(outputCloud2);
    tableFilter.setFilterLimits(-0.6, 0.55); //Simulation
    //tableFilter.setFilterLimits(-0.6, 0.55); //Real
    tableFilter.setFilterFieldName("x");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud3(new pcl::PointCloud<pcl::PointXYZRGB>);
    tableFilter.filter(*outputCloud3);

    /* Downsampling????????????????????????????? */

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoRGB(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : outputCloud3->points)
    {
        pcl::PointXYZ newPoint;
        newPoint.x = point.x;
        newPoint.y = point.y;
        newPoint.z = point.z;
        cloudNoRGB->points.push_back(newPoint);
    }

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBtoXYZHSV(*outputCloud3, *hsvCloud);

    /* RGB??? */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZHSV> euclideanClusters;
    euclideanClusters.setInputCloud(hsvCloud);
    euclideanClusters.setMinClusterSize(50);
    euclideanClusters.setClusterTolerance(0.065);
    euclideanClusters.extract(cluster_indices);

    /*
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusters;
    euclideanClusters.setInputCloud(cloudNoRGB);
    euclideanClusters.setMinClusterSize(50);
    euclideanClusters.setClusterTolerance(0.065);
    euclideanClusters.extract(cluster_indices);
    */

    std::stringstream output;
    output << "Clusters: " << cluster_indices.size() << std::endl;
    ROS_INFO("%s", output.str().c_str());

    int colours[] = 
    {
        100, 0, 0,
        0, 100, 0,
        0, 0, 100,
        255, 0, 0,
        0, 255, 0,
        0, 0, 255,
        255, 255, 0,
        0, 255, 255,
        255, 0, 255,
        60, 60, 60,
        128, 128, 128,
        255, 255, 255
    };

    
    int j = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr completeCloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            pcl::PointXYZRGB point;
            point.x = hsvCloud->points[*pit].x;
            point.y = hsvCloud->points[*pit].y;
            point.z = hsvCloud->points[*pit].z;
            point.r = colours[j];
            point.g = colours[j+1];
            point.b = colours[j+2];
            completeCloud->points.push_back (point);
        }
        j += 3;
    }

    /*
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(completeCloud);
    while (!viewer.wasStopped ())
    {
    }
    */
    
    
    std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr> detections;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> detectionsNoHSV;

    j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        detections.push_back(pcl::PointCloud<pcl::PointXYZHSV>::Ptr(new pcl::PointCloud<pcl::PointXYZHSV>));
        detectionsNoHSV.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            detections[j]->points.push_back(hsvCloud->points[*pit]);
            pcl::PointXYZ point;
            point.x = hsvCloud->points[*pit].x;
            point.y = hsvCloud->points[*pit].y;
            point.z = hsvCloud->points[*pit].z;
            detectionsNoHSV[j]->points.push_back(point);
        }
        ++j;
    }
    

    for (int i = 0; i < detections.size(); ++i)
    {
        /*
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr testCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto &point :detections[i]->points)
        {
            pcl::PointXYZRGB testPoint;
            testPoint.x = point.x;
            testPoint.y = point.y;
            testPoint.z = point.z;
            if (point.h == 0) 
            {
                testPoint.r = 255;
                testPoint.g = 0;
                testPoint.b = 0;
                ROS_INFO("Bad point: (%f, %f, %f)", point.x, point.y, point.z);
            }
            else if (point.h == 60)
            {
                testPoint.r = 0;
                testPoint.g = 255;
                testPoint.b = 0;
            }
            else
            {
                testPoint.r = 0;
                testPoint.g = 0;
                testPoint.b = 255;
            }
            testCloud->points.push_back(testPoint);
        }
        pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        viewer.showCloud(testCloud);
        while (!viewer.wasStopped ())
        {
        }
        */

        ROS_INFO("Detection %d", i);
        float averageHue = 0.0;
        int pointCount = 0;
        for (const auto &point : detections[i]->points)
        {
            if (point.s > 0.1)
            { 
                averageHue += point.h;
                ++pointCount;
            }
        }
        // TODO: Check divisione per 0
        ROS_INFO("Number of points: %d", detections[i]->points.size());
        averageHue /= pointCount;
        ROS_INFO("Average hue: %f", averageHue);
        
        DetectionObject detectionChoice;
        if (averageHue > 30 && averageHue <= 90)
        {
            detectionChoice = {Mesh::HEX, Colour::YELLOW};
            //Esagono
        }
        else if (averageHue > 90 && averageHue <= 180)
        {
            detectionChoice = {Mesh::PRISM, Colour::GREEN};
            //Prisma verde
        }
        else if (averageHue > 180 && averageHue <= 270)
        {
            detectionChoice = {Mesh::CUBE, Colour::BLUE};
            //Cubo blu
        }
        else
        {
            //Cubo o prisma rossi
            float maxHeight = 200;
            for (const auto &point : detections[i]->points)
            {
                if (point.z < maxHeight) maxHeight = point.z;
            }
            if (maxHeight <= 1.9107) 
            {
                detectionChoice = {Mesh::CUBE, Colour::RED};
            }
            else 
            {
                detectionChoice = {Mesh::PRISM, Colour::RED};
            }
        }

        /*
        pcl::PointCloud<pcl::PointXYZ>::Ptr translatedReference (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &point : _objects[static_cast<int>(detectionChoice.mesh)]->points)
        {
            pcl::PointXYZ translatedPoint;
            translatedPoint.x = point.x + detectionsNoHSV[i]->points[0].x;
            translatedPoint.y = point.y + detectionsNoHSV[i]->points[0].y;
            translatedPoint.z = point.z + detectionsNoHSV[i]->points[0].z;
            translatedReference->points.push_back(translatedPoint);
        }
        */

        pcl::PointCloud<pcl::PointXYZ>::Ptr lastCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(_objects[static_cast<int>(detectionChoice.mesh)]);
        icp.setInputTarget(detectionsNoHSV[i]);
        icp.setEuclideanFitnessEpsilon(_icp_fitness_epsilon);
        icp.setTransformationEpsilon(_icp_transformation_epsilon);
        icp.setMaxCorrespondenceDistance(_icp_correspondence_distance);
        icp.setRANSACIterations(_icp_ransac_iterations);
        icp.setRANSACOutlierRejectionThreshold(_icp_inlier_threshold);
        //auto convCrit = icp.getConvergeCriteria();
        //convCrit->setAbsoluteMSE(_icp_absolute_mse);
        //convCrit->setTranslationThreshold(_icp_translation_threshold);
        //convCrit->setRotationThreshold(_icp_rotation_threshold);
        //convCrit->setMaximumIterations(_icp_max_iterations);
        //convCrit->setMaximumIterationsSimilarTransforms(_icp_max_similar_iterations);
        //convCrit->setRelativeMSE(_icp_relative_mse);
        icp.align(*lastCloud);

        
        for (const auto &point : lastCloud->points)
        {
            pcl::PointXYZRGB modelP;
            modelP.x = point.x;
            modelP.y = point.y;
            modelP.z = point.z;
            modelP.r = 195;
            modelP.g = 0;
            modelP.b = 252;
            outputCloud3->points.push_back(modelP);
        }
        

        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        float x = transformation(0,3);
        float y = transformation(1,3);
        float z = transformation(2,3);

        std::cout << transformation(0,0) << std::endl;
        float fiX = atan2(transformation(2,1), transformation(2,2));
        float fiY = atan2(-transformation(2,0), sqrt(pow(transformation(2,1), 2) + pow(transformation(2,2), 2)));
        float fiZ = atan2(transformation(1,0), transformation(0,0));

        std::cout << "Angles: " << fiX << " " << fiY << " " << fiZ << std::endl;

        tf::Matrix3x3 rotationMatrix;
        rotationMatrix.setValue(transformation(0,0), transformation(0,1), transformation(0,2),
                                transformation(1,0), transformation(1,1), transformation(1,2),
                                transformation(2,0), transformation(2,1), transformation(2,2));
        tf::Quaternion quat;
        rotationMatrix.getRotation(quat);
        std::cout << "Quaternion " << quat.getX() << " " << quat.getY() << " " << quat.getZ() << " " << quat.getW() << std::endl;

        std::cout << transformation << std::endl;


        /*
        pcl::PointCloud<pcl::PointXYZ>::Ptr lastCloud (new pcl::PointCloud<pcl::PointXYZ>);
        float maxHeight = 0;
        double minScore = 5;
        Mesh matchingMesh;
        for (const auto &reference : detectionChoices)
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(_objects[static_cast<int>(reference.mesh)]);
            icp.setInputTarget(detectionsNoHSV[i]);

            icp.align(*lastCloud);
            if (icp.getFitnessScore() < minScore)
            {
                minScore = icp.getFitnessScore();
                matchingMesh = reference.mesh;
            }
        }
        */
        
        /*
        j = 0;
        for (const auto &reference : _objects)
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(detectionsNoHSV[i]);
            icp.setInputTarget(reference);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr lastCloud (new pcl::PointCloud<pcl::PointXYZ>);
            icp.align(*lastCloud);
            std::cout << "Has converged? " << icp.hasConverged() << std::endl;
            std::cout << "Source mesh: " << PATHS[j].c_str() << "; Match: " << icp.getFitnessScore() << std::endl;
            ++j;
        }
        */
        
        
        /*std::cout << "Source mesh: " << PATHS[static_cast<int>(matchingMesh)].c_str() << "; Match: " << minScore << std::endl;*/

        /*
        pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        viewer.showCloud(_objects[static_cast<int>(matchingMesh)]);
        while (!viewer.wasStopped ())
        {
        }
        */
    }

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(outputCloud3);
    while (!viewer.wasStopped ())
    {
    }
}