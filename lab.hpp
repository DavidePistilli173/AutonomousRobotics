#ifndef LAB_HPP
#define LAB_HPP

/* Useful functions and constants for all packages. */
namespace lab
{
    /********** STRUCTS AND ENUMS **********/
    struct Quaternion
    {
        double w;
        double x;
        double y;
        double z;
    };
    /* Mesh types. */
    enum class Mesh
    {
        CUBE,
        HEX,
        PRISM
    };
    /* Generic status. */
    enum class Status
    {
        SUCCESS,
        PARTIAL_FAILURE,
        FAILURE
    };

    /********** CONSTANTS **********/
    constexpr double PI = 3.141592653589793;
    constexpr int N = 16; // Maximum number of objects in the scene.
    constexpr int Q_LEN = 1000; // Queue length for ROS messages.
    constexpr double MESH_HEIGHTS[] = {0.091, 0.192, 0.015}; // hex 0.193 ottimo tranne che in un angolo
                                                             // probabilmente 0.020 va bene

    /********** FUNCTIONS **********/
    /* Return the rotation around the z axis.*/
    double getZAngle(Quaternion q);
}

#endif