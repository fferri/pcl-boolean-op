#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/parsers.hpp>

#ifndef M_2PI
#define M_2PI (2 * M_PI)
#endif

bool verbose = false;
int subdivisions = 64;

template<typename PointA, typename PointB>
inline double dist(const PointA& a, const PointB& b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

inline size_t flatBucket(const size_t bucket1, const size_t bucket2, const size_t subdivisions)
{
    return bucket1 * subdivisions + bucket2;
}

inline void angleToBucket(double theta, double phi, size_t& bucket1, size_t& bucket2, const size_t subdivisions)
{
    // normalize angles between 0,2PI
    while(phi < 0) phi += M_2PI;
    while(theta < 0) theta += M_PI;

    bucket1 = (size_t)(phi * subdivisions / M_2PI);
    bucket2 = (size_t)(theta * subdivisions / M_PI);
}

inline void bucketToAngle(const size_t bucket1, const size_t bucket2, double& theta, double& phi, const size_t subdivisions)
{
    phi = bucket1 * M_2PI / (double)subdivisions;
    theta = bucket2 * M_PI / (double)subdivisions;
}

template<typename T>
inline void toEuclidean(double r, double theta, double phi, T& x, T& y, T& z, double offsetX = 0, double offsetY = 0, double offsetZ = 0)
{
    x = offsetX + r * sin(theta) * cos(phi);
    y = offsetY + r * sin(theta) * sin(phi);
    z = offsetZ + r * cos(theta);
}

inline void toSpherical(double x, double y, double z, double& r, double& theta, double& phi, double offsetX = 0, double offsetY = 0, double offsetZ = 0)
{
    // radius [0..infty):
    r = sqrt(pow(x - offsetX, 2) + pow(y - offsetY, 2) + pow(z - offsetZ, 2));
    // inclination [0..PI]:
    theta = acos((z - offsetZ) / r);
    //  azimuth [0..2PI):
    phi = atan2(y - offsetY, x - offsetX);
}

template<typename PointT>
void sphericalPartition(const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT> *output, const Eigen::Vector4f& center, unsigned int subdivisions)
{
    // partition according to phi,theta:
    for(size_t i = 0; i < input.size(); i++)
    {
        const PointT& p = input.points[i];
        double r, theta, phi;
        toSpherical(p.x, p.y, p.z, r, theta, phi, center.x(), center.y(), center.z());
        size_t bucket1 = 0, bucket2 = 0;
        angleToBucket(theta, phi, bucket1, bucket2, subdivisions);
        output[flatBucket(bucket1, bucket2, subdivisions)].push_back(p);
    }
}

template<typename PointT>
void removeDynamicObstacles(const pcl::PointCloud<PointT>& s, const pcl::PointCloud<PointT>& scan, Eigen::Vector4f scan_pos, Eigen::Quaternionf scan_orient, pcl::PointCloud<PointT>& result, int num_subdivisions)
{
    // remove outliers from scan:
    pcl::PointCloud<PointT> scan_f;
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(scan.makeShared());
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(2);
    ror.filter(scan_f);

    pcl::PointXYZ scan_pos_p;
    scan_pos_p.x = scan_pos.x();
    scan_pos_p.y = scan_pos.y();
    scan_pos_p.z = scan_pos.z();

    pcl::PointCloud<PointT> scan_partition[num_subdivisions * num_subdivisions];
    pcl::PointCloud<PointT> s_partition[num_subdivisions * num_subdivisions];

    sphericalPartition(scan_f, scan_partition, scan_pos, num_subdivisions);
    sphericalPartition(s, s_partition, scan_pos, num_subdivisions);

    for(size_t bucket1 = 0; bucket1 < num_subdivisions; bucket1++)
    {
        for(size_t bucket2 = 0; bucket2 < num_subdivisions; bucket2++)
        {
            size_t bucket = flatBucket(bucket1, bucket2, num_subdivisions);

            // find min distance:
            double min_dist = INFINITY;
            for(size_t i = 0; i < scan_partition[bucket].size(); i++)
                min_dist = std::min(min_dist, dist(scan_pos_p, scan_partition[bucket][i]));
            if(isinf(min_dist))
                min_dist = 0.0;

            // add points from map that do not fall into deletion volume:
            for(size_t j = 0; j < s_partition[bucket].size(); j++)
            {
                double d = dist(scan_pos_p, s_partition[bucket][j]);

                if(d >= min_dist)
                {
                    result.push_back(s_partition[bucket][j]);
                }
            }

            // add all points from scan:
            for(size_t j = 0; j < scan_partition[bucket].size(); j++)
            {
                result.push_back(scan_partition[bucket][j]);
            }
        }
    }
}

int main(int argc, char **argv)
{
    namespace po = boost::program_options;
    po::options_description desc("Usage: pcl_remove_dynamic_obstacles [options] s.pcd scan.pcd result.pcd\nOptions:");
    desc.add_options()
        ("subdivisions,s", po::value<int>(&subdivisions)->default_value(subdivisions), "number of subdivisions")
        ("verbose,v", "verbose mode")
        ("help,h", "print help")
    ;
    po::options_description hdesc("Hidden");
    hdesc.add_options()
        ("input-s,i", po::value<std::string>(), "s input file")
        ("input-scan,j", po::value<std::string>(), "scan input file")
        ("output,o", po::value<std::string>(), "output file")
    ;
    po::options_description desc_all;
    desc_all.add(desc).add(hdesc);
    po::positional_options_description p;
    p.add("input-s", 1);
    p.add("input-scan", 1);
    p.add("output", 1);
    po::variables_map vmap;
    po::store(po::command_line_parser(argc, argv).options(desc_all).positional(p).run(), vmap);
    po::notify(vmap);

    if(vmap.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    if(vmap.count("verbose"))
    {
        verbose = true;
    }

    Eigen::Vector4f t;
    Eigen::Quaternionf q;
    pcl::PCLPointCloud2 s2, scan2;
    pcl::PointCloud<pcl::PointXYZ> s, scan, result;

    std::string s_fn;
    std::string scan_fn;
    std::string result_fn;
    try
    {
        s_fn = vmap["input-s"].as<std::string>();
        scan_fn = vmap["input-scan"].as<std::string>();
        result_fn = vmap["output"].as<std::string>();
    }
    catch(boost::bad_any_cast& ex)
    {
        std::cout << desc << std::endl;
        exit(1);
    }

    if(vmap.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    if(pcl::io::loadPCDFile(s_fn, s2) == -1)
    {
        std::cerr << "error: load of " << s_fn << " failed" << std::endl;
        return 1;
    }
    pcl::fromPCLPointCloud2(s2, s);
    if(verbose)
        std::cout << "info: loaded " << s.size() << " points from " << s_fn << std::endl;

    if(pcl::io::loadPCDFile(scan_fn, scan2, t, q) == -1)
    {
        std::cerr << "error: load of " << scan_fn << " failed" << std::endl;
        return 1;
    }
    pcl::fromPCLPointCloud2(scan2, scan);
    if(verbose)
        std::cout << "info: loaded " << scan.size() << " points from " << scan_fn << std::endl;

    removeDynamicObstacles(s, scan, t, q, result, subdivisions);

    if(pcl::io::savePCDFile<pcl::PointXYZ>(result_fn, result) == -1)
    {
        std::cerr << "error: write of " << result_fn << " failed" << std::endl;
        return 1;
    }
    if(verbose)
        std::cout << "info: wrote " << result.size() << " points to " << result_fn << std::endl;
}

