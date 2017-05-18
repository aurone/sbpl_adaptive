#ifndef SBPL_ADAPTIVE_COMMON_H
#define SBPL_ADAPTIVE_COMMON_H

// standard includes
#include <math.h>
#include <chrono>
#include <iomanip>
#include <ostream>
#include <string>

// system includes
#include <leatherman/utils.h>
#include <smpl/time.h>
#include <std_msgs/ColorRGBA.h>

namespace adim {

struct StopWatch_t
{
    sbpl::clock::time_point start_t;

    StopWatch_t();
    double getElapsedSeconds();
    void reset();
    void print(const std::string &prefix);
};

struct AdaptiveSphere2D
{
    int dimID;
    int costToGoal;
    double x;
    double y;
    double rad;
    double near_rad;
};

inline std::ostream &operator<<(std::ostream &o, const AdaptiveSphere2D &s)
{
    o << "{ dim: " << s.dimID <<
            ", cost_to_goal: " << s.costToGoal <<
            ", x: " << std::fixed << std::setprecision(3) << s.x <<
            ", y: " << std::fixed << std::setprecision(3) << s.y <<
            ", rad: " << std::fixed << std::setprecision(3) << s.rad <<
            ", near_rad: " << std::fixed << std::setprecision(3) << s.near_rad <<
            " }";
    return o;
}

struct AdaptiveSphere3D
{
    int dimID;
    int costToGoal;
    double x, y, z, rad, near_rad;
};

inline std::ostream &operator<<(std::ostream &o, const AdaptiveSphere3D &s)
{
    o << "{ dim: " << s.dimID <<
            ", cost_to_goal: " << s.costToGoal <<
            ", x: " << std::fixed << std::setprecision(3) << s.x <<
            ", y: " << std::fixed << std::setprecision(3) << s.y <<
            ", z: " << std::fixed << std::setprecision(3) << s.z <<
            ", rad: " << std::fixed << std::setprecision(3) << s.rad <<
            ", near_rad: " << std::fixed << std::setprecision(3) << s.near_rad <<
            " }";
    return o;
}

class AbstractGoal
{
};

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

} // namespace adim

#endif
