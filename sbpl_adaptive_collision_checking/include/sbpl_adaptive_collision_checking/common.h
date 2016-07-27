/*
 * common.h
 *
 *  Created on: Mar 30, 2016
 *      Author: kalin
 */

#ifndef SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_COMMON_H_
#define SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_COMMON_H_

namespace sbpl_adaptive_collision_checking {

inline static std_msgs::ColorRGBA fromHSV(double h, double s, double v)
{
    std_msgs::ColorRGBA out;
    int i;
    double f, p, q, t;
    if (s == 0) {
        // achromatic (grey)
        out.r = out.g = out.b = v;
        out.a = 1.0;
        return out;
    }
    h /= 60;        // sector 0 to 5
    i = floor(h);
    f = h - i;          // factorial part of h
    p = v * (1 - s);
    q = v * (1 - s * f);
    t = v * (1 - s * (1 - f));
    switch (i) {
    case 0:
        out.r = v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = v;
        out.b = t;
        break;
    case 3:
        out.r = p;
        out.g = q;
        out.b = v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = v;
        break;
    default:
        out.r = v;
        out.g = p;
        out.b = q;
        break;
    }
    out.a = 1.0;
    return out;
}

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
