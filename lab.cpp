#include "lab.hpp"

#include <math.h>

namespace lab
{
    double getZAngle(Quaternion q)
    {
        if (q.w == 1) return 0.0; // No rotation.

        double qAngle = acos(q.w); // Half of the angle around q's rotation axis.
        double aX = q.x/sin(qAngle); // X component of the rotation axis.
        double aY = q.y/sin(qAngle); // Y component of the rotation axis.

        /* If the x component is 0, we have a rotation of either PI/2 or 3PI/2. */
        if (aX == 0)
        {
            if (aY > 0) return lab::PI/2;
            else if (aY < 0) return (3/2)*lab::PI;
            else return 0.0;
        }

        return 2 * atan(aY/aX);
    }
}