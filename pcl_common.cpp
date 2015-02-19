#include "pcl_common.h"
#include <sstream>

double tolerance = 1e-9;

void load_pcd(const char *filename, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(filename, cloud) == -1)
    {
        std::cerr << "error: load of " << filename << " failed" << std::endl;
        exit(1);
    }
    std::cout << "info: loaded " << cloud.size() << " points from " << filename << std::endl;
}

void save_pcd(const char *filename, const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if(pcl::io::savePCDFile<pcl::PointXYZ>(filename, cloud) == -1)
    {
        std::cerr << "error: save of " << filename << " failed" << std::endl;
        exit(1);
    }
    std::cout << "info: saved " << cloud.size() << " points to " << filename << std::endl;
}

point::point(double x, double y, double z) : x_(x), y_(y), z_(z)
{
}

bool point::operator<(const point& rhs) const
{
    if(ix() == rhs.ix())
    {
        if(iy() == rhs.iy())
        {
            return iz() < rhs.iz();
        }
        else
        {
            return iy() < rhs.iy();
        }
    }
    else
    {
        return ix() < rhs.ix();
    }
}

long point::ix() const
{
    return (long)round(x_ / tolerance);
}

long point::iy() const
{
    return (long)round(y_ / tolerance);
}

long point::iz() const
{
    return (long)round(z_ / tolerance);
}

double point::x() const
{
    return x_;
}

double point::y() const
{
    return y_;
}

double point::z() const
{
    return z_;
}

void usage(const char *self)
{
    std::cout << "usage: " << self << " [options] cloud_1.pcd ... cloud_N.pcd result.pcd" << std::endl
        << "options:" << std::endl
        << "  --tolerance <t>" << std::endl
        << "  -t <t>            two points are considered equal if:" << std::endl
        << "                    |x1-x2|<t && |y1-y2|<t && |z1-z2|<t" << std::endl;
}

void pcl_to_set(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::set<point>& s)
{
    for(int i = 0; i < cloud.size(); i++)
    {
        point p(cloud[i].x, cloud[i].y, cloud[i].z);
        s.insert(p);
    }
}

void set_to_pcl(const std::set<point>& s, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    for(std::set<point>::iterator it = s.begin(); it != s.end(); ++it)
    {
        pcl::PointXYZ p;
        p.x = it->x();
        p.y = it->y();
        p.z = it->z();
        cloud.push_back(p);
    }
}

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cout << "error: need at least 3 point clouds" << std::endl;
        usage(argv[0]);
        exit(1);
    }

    int i = 1;

    if(strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--tolerance") == 0)
    {
        tolerance = atof(argv[i + 1]);
        std::cout << "info: tolerance set to " << tolerance << std::endl;
        i += 2;
    }
    else
    {
        std::cout << "info: using default tolerance of " << tolerance << std::endl;
    }

    if(argc < (i + 2 + 1))
    {
        std::cout << "error: need at least 3 point clouds" << std::endl;
        usage(argv[0]);
        exit(1);
    }

    pcl::PointCloud<pcl::PointXYZ> tmp;
    std::set<point> a, b, c;
    std::string namea, nameb, namec;

    bool first_operand = true;

    load_pcd(argv[i], tmp);
    namea = argv[i];
    pcl_to_set(tmp, a);

    for(int j = i + 1; j < (argc - 1); j++)
    {
        if(!first_operand)
        {
            c.swap(a);
            namea = namec;
        }

        b.clear();
        c.clear();
        tmp.clear();

        load_pcd(argv[j], tmp);
        nameb = argv[j];
        pcl_to_set(tmp, b);

        op(a, b, c);

        std::stringstream namec_ss;
        namec_ss << "pcl_" << (j - i);
        namec = namec_ss.str();

        std::cout << namea << " * " << nameb << " = " << namec << " (" << c.size() << " points)" << std::endl;

        first_operand = false;
    }

    std::cout << "result is " << namec << std::endl;

    tmp.clear();
    set_to_pcl(c, tmp);
    save_pcd(argv[argc - 1], tmp);
}


