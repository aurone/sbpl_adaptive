/*
 * common.h
 *
 *  Created on: Sep 22, 2015
 *      Author: kalin
 */

#ifndef _SBPL_ADAPTIVE_COMPONENTS_COMMON_H_
#define _SBPL_ADAPTIVE_COMPONENTS_COMMON_H_

#include <tf/tf.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace sbpl_adaptive_components {

typedef struct {
	double x;
	double y;
	double z;
} Point3D_t;

typedef struct {
	int x;
	int y;
	int z;
} Cell3D_t;

typedef struct {
	double x;
	double y;
	double z;
	double rad;
} Sphere3D_t;

typedef struct {
	int x;
	int y;
	int z;
	int rad;
} CellSphere3D_t;

typedef struct ColorRGBA_t{
	double r;
	double g;
	double b;
	double a;

	ColorRGBA_t(double r_, double g_, double b_) { r = r_; g = g_; b = b_; a = 1; }
	ColorRGBA_t(double r_, double g_, double b_, double a_) { r = r_; g = g_; b = b_; a = a_; }
} Color_t;

inline static Color_t colorFromHSV(double h, double s, double v){
	int i;
	double r, g, b;
	double f, p, q, t;
	if( s == 0 ) {
		// achromatic (grey)
		r = g = b = v;
		return Color_t(r, g, b);
	}
	h /= 60;        // sector 0 to 5
	i = floor(h);
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			r = v;
			g = t;
			b = p;
			break;
		case 1:
			r = q;
			g = v;
			b = p;
			break;
		case 2:
			r = p;
			g = v;
			b = t;
			break;
		case 3:
			r = p;
			g = q;
			b = v;
			break;
		case 4:
			r = t;
			g = p;
			b = v;
			break;
	default:
			r = v;
			g = p;
			b = q;
			break;
	}
	return Color_t(r, g, b);
}

inline static std_msgs::ColorRGBA toColorMSG(Color_t col){
	std_msgs::ColorRGBA c;
	c.r = col.r;
	c.g = col.g;
	c.b = col.b;
	c.a = col.a;
	return c;
}

typedef struct VizLabel_t_
{
	std::string label;
	double fontsize;
	tf::Vector3 offset;
public:
	VizLabel_t_(std::string l, double s, tf::Vector3 off){
		offset = off;
		label = l;
		fontsize = s;
	}
} VizLabel_t;

}

#endif /* SRC_MRSLC_PLANNER_SBPL_ADAPTIVE_COMPONENTS_INCLUDE_SBPL_ADAPTIVE_COMPONENTS_COMMON_H_ */
