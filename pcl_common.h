#ifndef PCL_COMMON_H_INCLUDED
#define PCL_COMMON_H_INCLUDED

#include <cmath>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

extern bool ignore_read_errors;
extern bool verbose;
extern double tolerance;
extern std::vector<std::string> input_filenames;
extern std::string output_filename;

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

void parse_args(int argc, char **argv);

void op(std::set<point>& a, std::set<point>& b, std::set<point>& c);

#endif // PCL_COMMON_H_INCLUDED
