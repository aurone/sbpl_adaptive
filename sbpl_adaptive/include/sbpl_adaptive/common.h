/*
 * common.h
 *
 *  Created on: Mar 15, 2016
 *      Author: kalin
 */

#ifndef SBPL_ADAPTIVE_COMMON_H
#define SBPL_ADAPTIVE_COMMON_H

#include <math.h>
#include <chrono>
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

struct AdaptiveSphere2D
{
    int dimID;
    int costToGoal;
    double x, y, rad, near_rad;
};

struct AdaptiveSphere3D
{
    int dimID;
    int costToGoal;
    double x, y, z, rad, near_rad;

    static void print(const AdaptiveSphere3D &sphere);
};

class AbstractGoal
{
};

/** @brief struct that describes a basic pose constraint */
struct Position3D
{
    double x, y, z;

    Position3D() : x(0), y(0), z(0) { }
    Position3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) { }

    static double dist(const Position3D &ee1, const Position3D &ee2);
};

struct Orientation3D
{
    double x, y, z, w;

    Orientation3D() : x(0), y(0), z(0), w(1) { }
    Orientation3D(double x_, double y_, double z_, double w_) :
        x(x_), y(y_), z(z_), w(w_) { }
};

class AbstractGoal3D : public AbstractGoal
{
public:

    AbstractGoal3D(Position3D p, double tol) : pos(p), pos_tol(tol) { }

    virtual ~AbstractGoal3D() { };

    virtual bool isSatisfied(Position3D p) const = 0;

    const Position3D &getPosition() const { return pos; }

    double getPositionTolerance() const { return pos_tol; }

protected:

    Position3D pos;
    double pos_tol;
};

class AbstractGoal6D : public AbstractGoal3D
{
public:

    AbstractGoal6D(
        Position3D p,
        double p_tol,
        Orientation3D o,
        double o_tol)
    :
        AbstractGoal3D(p, p_tol), ori(o), ori_tol(o_tol)
    { }

    virtual ~AbstractGoal6D() { };

    virtual bool isSatisfied(Orientation3D ori) const  = 0;

    virtual bool isSatisfied(Position3D pos, Orientation3D ori) const = 0;

    const Orientation3D &getOrientation() const { return ori; }

    double getOrientationTolerance() const { return ori_tol; }

protected:

    Orientation3D ori;
    double ori_tol;
};

inline
double Position3D::dist(const Position3D &ee1, const Position3D &ee2)
{
    const double dx = ee1.x - ee2.x;
    const double dy = ee1.y - ee2.y;
    const double dz = ee1.z - ee2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void pause();
bool prompt();

template <class Rep, class Period>
double to_secs(const std::chrono::duration<Rep, Period>& d)
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
}

} // namespace adim

#endif /* SRC_SBPL_ADAPTIVE_INCLUDE_SBPL_ADAPTIVE_COMMON_H_ */
