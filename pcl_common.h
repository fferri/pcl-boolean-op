#include <cmath>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class point
{
private:
    double x_;
    double y_;
    double z_;
public:
    point(double x_, double y_, double z_);
    bool operator<(const point& rhs) const;
    long ix() const;
    long iy() const;
    long iz() const;
    double x() const;
    double y() const;
    double z() const;
};

void op(std::set<point>& a, std::set<point>& b, std::set<point>& c);
