/*
 * common.h
 *
 *  Created on: Sep 22, 2015
 *      Author: kalin
 */

#ifndef _SBPL_ADAPTIVE_COMPONENTS_COMMON_H_
#define _SBPL_ADAPTIVE_COMPONENTS_COMMON_H_

#include <leatherman/utils.h>
#include <tf/tf.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace adim {

struct Point3D
{
   double x;
   double y;
   double z;
};

struct Cell3D
{
    int x;
    int y;
    int z;
};

struct Sphere3D
{
    double x;
    double y;
    double z;
    double rad;
};

struct CellSphere3D
{
    int x;
    int y;
    int z;
    int rad;
};

struct Color
{
    double r;
    double g;
    double b;
    double a;

    Color(double r_, double g_, double b_) : r(r_), g(g_), b(b_), a(1) { }
    Color(double r_, double g_, double b_, double a_) :
        r(r_), g(g_), b(b_), a(a_) { }
};

inline
Color colorFromHSV(double h, double s, double v)
{
    double r, g, b;
    leatherman::HSVtoRGB(&r, &g, &b, h, s, v);
    return Color(r, g, b);
}

inline
std_msgs::ColorRGBA toColorMSG(Color col)
{
    std_msgs::ColorRGBA c;
    c.r = col.r;
    c.g = col.g;
    c.b = col.b;
    c.a = col.a;
    return c;
}

inline
std_msgs::ColorRGBA fromHSV(double h, double s, double v)
{
    double r, g, b;
    leatherman::HSVtoRGB(&r, &g, &b, h, s, v);
    std_msgs::ColorRGBA col;
    col.r = r;
    col.g = g;
    col.b = b;
    col.a = 1.0;
    return col;
}

struct VizLabel
{
    std::string label;
    double fontsize;
    tf::Vector3 offset;

    VizLabel(const std::string& l, double s, const tf::Vector3& off) :
        label(l), fontsize(s), offset(off) { }
};

} // sbpl_adaptive_components

#endif
