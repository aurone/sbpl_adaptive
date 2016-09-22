/*
 * interpolation.h
 *
 *  Created on: Mar 15, 2016
 *      Author: kalin
 */

#ifndef SRC_SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_INTERPOLATION_H_
#define SRC_SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_INTERPOLATION_H_

#include <math.h>
#include <angles/angles.h>

namespace adim {

inline static double LERP(double v0, double v1, double t)
{
    return (1.0 - t) * v0 + t * v1;
}

inline static double ALERP(double a0, double a1, double t)
{
    double a0n = angles::normalize_angle(a0);
    double a1n = angles::normalize_angle(a1);
    double a_diff = a1n - a0n;
    if (a_diff > M_PI) { //very positive --> make negative and rotate the other way
        a_diff = -(2.0 * M_PI - a_diff);
    }
    else if (a_diff < -M_PI) { //very negative --> make positive and rotate the other way
        a_diff = (2.0 * M_PI + a_diff);
    }
    return angles::normalize_angle(a0 + t * a_diff);
}

} // namespace adim

#endif /* SRC_SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_INTERPOLATION_H_ */
