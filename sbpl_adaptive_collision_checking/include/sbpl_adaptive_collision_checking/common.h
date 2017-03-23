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
};

} // namespace adim

#endif
