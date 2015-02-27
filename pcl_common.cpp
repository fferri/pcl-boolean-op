#include "pcl_common.h"
#include <sstream>

#include <boost/program_options.hpp>
#include <boost/program_options/parsers.hpp>

bool ignore_read_errors = true;
bool verbose = false;
double tolerance = 1e-9;
std::vector<std::string> input_filenames;
std::string output_filename;

void load_pcd(const char *filename, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(filename, cloud) == -1 && !ignore_read_errors)
    {
        if(ignore_read_errors)
        {
            cloud.clear();
        }
        else
        {
            std::cerr << "error: load of " << filename << " failed" << std::endl;
            exit(1);
        }
    }
    if(verbose)
        std::cout << "info: loaded " << cloud.size() << " points from " << filename << std::endl;
}

void save_pcd(const char *filename, const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if(pcl::io::savePCDFile<pcl::PointXYZ>(filename, cloud) == -1)
    {
        std::cerr << "error: save of " << filename << " failed" << std::endl;
        exit(1);
    }
    if(verbose)
        std::cout << "info: saved " << cloud.size() << " points to " << filename << std::endl;
}

point::point(double x, double y, double z)
{
    ix_ = (long)round(x / tolerance);
    iy_ = (long)round(y / tolerance);
    iz_ = (long)round(z / tolerance);
}

bool point::operator<(const point& rhs) const
{
    if(ix_ == rhs.ix_)
    {
        if(iy_ == rhs.iy_)
        {
            return iz_ < rhs.iz_;
        }
        else
        {
            return iy_ < rhs.iy_;
        }
    }
    else
    {
        return ix_ < rhs.ix_;
    }
}

double point::x() const
{
    return ix_ * tolerance;
}

double point::y() const
{
    return iy_ * tolerance;
}

double point::z() const
{
    return iz_ * tolerance;
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

void parse_args(int argc, char **argv)
{
    namespace po = boost::program_options;
    std::string usage = "Usage: " + std::string(argv[0]) + " [options] input_1.pcd input_2.pcd ... input_N.pcd result.pcd";
    po::options_description desc(usage + "\nOptions:");
    desc.add_options()
        ("tolerance,t", po::value<double>(&tolerance)->default_value(tolerance), "tolerance ( = voxel size)")
        ("stop-on-errors,S", "stop when a readnig error occurs")
        ("verbose,v", "verbose mode")
        ("help,h", "print help")
    ;
    po::options_description hdesc("Hidden");
    hdesc.add_options()
        ("file,f", po::value<std::vector<std::string> >(), "file")
    ;
    po::options_description desc_all;
    desc_all.add(desc).add(hdesc);
    po::positional_options_description p;
    p.add("file", -1);
    po::variables_map vmap;
    po::store(po::command_line_parser(argc, argv).options(desc_all).positional(p).run(), vmap);
    po::notify(vmap);

    if(vmap.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    if(vmap.count("stop-on-errors"))
    {
        ignore_read_errors = false;
    }

    if(vmap.count("verbose"))
    {
        verbose = true;
    }

    try
    {
        std::vector<std::string> filenames = vmap["file"].as<std::vector<std::string> >();
        if(filenames.size() < 3)
        {
            std::cout << "error: need at least 3 file arguments (got " << filenames.size() << ")" << std::endl << desc << std::endl;
            exit(1);
        }
        input_filenames.resize(filenames.size() - 1);
        std::copy(filenames.begin(), filenames.begin() + filenames.size() - 1, input_filenames.begin());
        output_filename = filenames[filenames.size() - 1];
    }
    catch(boost::bad_any_cast& ex)
    {
        std::cout << desc << std::endl;
        exit(1);
    }

    if(verbose)
    {
        if(vmap.count("tolerance"))
            std::cout << "info: tolerance set to " << tolerance << std::endl;
        else
            std::cout << "info: using default tolerance of " << tolerance << std::endl;
    }

    if(input_filenames.size() < 2)
    {
        std::cout << "error: need at least 2 input clouds and 1 output cloud" << std::endl << desc << std::endl;
        exit(1);
    }
}

