/*
 * common.h
 *
 *  Created on: Mar 15, 2016
 *      Author: kalin
 */

#ifndef SRC_SBPL_ADAPTIVE_INCLUDE_SBPL_ADAPTIVE_COMMON_H_
#define SRC_SBPL_ADAPTIVE_INCLUDE_SBPL_ADAPTIVE_COMMON_H_

#include <math.h>
#include <string>

#define ADP_USE_ROS_TIMERS //use ros::Time for timing

#if defined(ROS_TIME_H_INCLUDED) && defined(ADP_USE_ROS_TIMERS)
    #define MY_TIME_TYPE double
    #define MY_TIME_NOW ros::Time::now().toSec()
    #define MY_TIME_DIFF_S(now,last) (now - last)
#else
    #define MY_TIME_TYPE int
    #define MY_TIME_NOW clock()
    #define MY_TIME_DIFF_S(now,last) ((now - last)/(double) CLOCKS_PER_SEC)
#endif

#define MY_TIME_ELAPSED_S(since) (MY_TIME_DIFF_S(MY_TIME_NOW, since))

namespace adim {

struct StopWatch_t
{
    MY_TIME_TYPE start_t;

    StopWatch_t();
    double getElapsedSeconds();
    void reset();
    void print(const std::string &prefix);
};

struct AdaptiveSphere2D_t
{
    int dimID;
    int costToGoal;
    double x, y, rad, near_rad;
};

struct AdaptiveSphere3D_t
{
    int dimID;
    int costToGoal;
    double x, y, z, rad, near_rad;

    static void print(const AdaptiveSphere3D_t &sphere);
};

class AbstractGoal_t
{
};

/** @brief struct that describes a basic pose constraint */
struct Position3D_t
{
    double x, y, z;

    Position3D_t() : x(0), y(0), z(0) { }
    Position3D_t(double x_, double y_, double z_) : x(x_), y(y_), z(z_) { }

    static double dist(const Position3D_t &ee1, const Position3D_t &ee2);
};

struct Orientation3D_t
{
    double x, y, z, w;

    Orientation3D_t() : x(0), y(0), z(0), w(1) { }
    Orientation3D_t(double x_, double y_, double z_, double w_) :
        x(x_), y(y_), z(z_), w(w_) { }
};

class AbstractGoal3D_t : public AbstractGoal_t
{
public:

    AbstractGoal3D_t(Position3D_t p, double tol) : pos(p), pos_tol(tol) { }

    virtual ~AbstractGoal3D_t() { };

    virtual bool isSatisfied(Position3D_t p) const = 0;

    const Position3D_t &getPosition() const { return pos; }

    double getPositionTolerance() const { return pos_tol; }

protected:

    Position3D_t pos;
    double pos_tol;
};

class AbstractGoal6D_t : public AbstractGoal3D_t
{
public:

    AbstractGoal6D_t(
        Position3D_t p,
        double p_tol,
        Orientation3D_t o,
        double o_tol)
    :
        AbstractGoal3D_t(p, p_tol), ori(o), ori_tol(o_tol)
    { }

    virtual ~AbstractGoal6D_t() { };

    virtual bool isSatisfied(Orientation3D_t ori) const  = 0;

    virtual bool isSatisfied(Position3D_t pos, Orientation3D_t ori) const = 0;

    const Orientation3D_t &getOrientation() const { return ori; }

    double getOrientationTolerance() const { return ori_tol; }

protected:

    Orientation3D_t ori;
    double ori_tol;
};

inline
double Position3D_t::dist(const Position3D_t &ee1, const Position3D_t &ee2)
{
    const double dx = ee1.x - ee2.x;
    const double dy = ee1.y - ee2.y;
    const double dz = ee1.z - ee2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace sbpl_adaptive

#endif /* SRC_SBPL_ADAPTIVE_INCLUDE_SBPL_ADAPTIVE_COMMON_H_ */
