#ifndef PCL_COMMON_H_INCLUDED
#define PCL_COMMON_H_INCLUDED

#include <cmath>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class point
{
private:
    long ix_;
    long iy_;
    long iz_;
public:
    point(double x_, double y_, double z_);
    bool operator<(const point& rhs) const;
    double x() const;
    double y() const;
    double z() const;
};

void load_pcd(const char *filename, pcl::PointCloud<pcl::PointXYZ>& cloud);

void save_pcd(const char *filename, const pcl::PointCloud<pcl::PointXYZ>& cloud);

void pcl_to_set(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::set<point>& s);

void set_to_pcl(const std::set<point>& s, pcl::PointCloud<pcl::PointXYZ>& cloud);

int parse_args(int argc, char **argv);

void op(std::set<point>& a, std::set<point>& b, std::set<point>& c);

#endif // PCL_COMMON_H_INCLUDED
