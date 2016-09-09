/*
 * common.h
 *
 *  Created on: Mar 30, 2016
 *      Author: kalin
 */

#ifndef SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_COMMON_H_
#define SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_COMMON_H_

namespace adim {

struct Sphere
{
    Eigen::Vector3d v;
    std::string link_name_;
    std::string name_;
    double radius;

    static void print(const Sphere &s)
    {
        printf("Sphere %s on %s [%.3f %.3f %.3f] rad=%.3f\n", s.name_.c_str(), s.link_name_.c_str(), s.v.x(), s.v.y(), s.v.z(), s.radius);
    }
};

} // namespace sbpl_adaptive_collision_checking

#endif /* SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_COMMON_H_ */
