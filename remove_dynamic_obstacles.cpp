#include "pcl_common.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/make_shared.hpp>

#ifndef M_2PI
#define M_2PI (2 * M_PI)
#endif

inline size_t flatBucket(const size_t bucket1, const size_t bucket2, const size_t subdivisions)
{
    return bucket1 * subdivisions + bucket2;
}

inline void angleToBucket(double theta, double phi, size_t& bucket1, size_t& bucket2, const size_t subdivisions)
{
    // normalize angles between 0,2PI
    while(phi < 0) phi += 2 * M_PI;
    while(theta < 0) theta += 2 * M_PI;

    bucket1 = (size_t)(phi * subdivisions / M_2PI);
    bucket2 = (size_t)(theta * subdivisions / M_PI);
}

inline void bucketToAngle(const size_t bucket1, const size_t bucket2, double& theta, double& phi, const size_t subdivisions)
{
    phi = bucket1 * M_2PI / (double)subdivisions;
    theta = bucket2 * M_PI / (double)subdivisions;
}

template<typename T>
inline void coordsSphericalToEuclidean(double r, double theta, double phi, T& x, T& y, T& z, double offsetX = 0, double offsetY = 0, double offsetZ = 0)
{
    x = offsetX + r * sin(theta) * cos(phi);
    y = offsetY + r * sin(theta) * sin(phi);
    z = offsetZ + r * cos(theta);
}

inline void coordsEuclideanToSpherical(double x, double y, double z, double& r, double& theta, double& phi, double offsetX = 0, double offsetY = 0, double offsetZ = 0)
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
        coordsEuclideanToSpherical(p.x, p.y, p.z, r, theta, phi, center.x, center.y, center.z);
        size_t bucket1 = 0, bucket2 = 0;
        angleToBucket(theta, phi, bucket1, bucket2, subdivisions);
        output[flatBucket(bucket1, bucket2, subdivisions)].push_back(p);
    }
}

template<typename PointT>
void removeDynamicObstacles(const pcl::PointCloud<PointT>& s, const pcl::PointCloud<PointT>& scan, Eigen::Vector4f scan_pos, Eigen::Quaternionf scan_orient, pcl::PointCloud<PointT>& result, int num_subdivisions)
{
    pcl::PointCloud<PointT> scan_partition[num_subdivisions * num_subdivisions],
        s_partition[num_subdivisions * num_subdivisions];

    sphericalPartition(scan, scan_partition, scan_pos, num_subdivisions);
    sphericalPartition(s, s_partition, scan_pos, num_subdivisions);

    for(size_t bucket1 = 0; bucket1 < num_subdivisions; bucket1++)
    {
        for(size_t bucket2 = 0; bucket2 < num_subdivisions; bucket2++)
        {
            size_t bucket = flatBucket(bucket1, bucket2, num_subdivisions);

            BucketVolume_MinDistance<Point_In, Point_Out> v_min(bucket1, bucket2, config);
            BucketVolume_Plane<Point_In, Point_Out> v_rsc(bucket1, bucket2, config);
            v_min.estimate(pcl_scan_partition[bucket], laser_center);
            BucketVolume<Point_In, Point_Out> *pv = &v_min;

            if(pcl_scan_partition[bucket].size() >= 4)
            {
                // we have enough points for fitting a plane...

                if(config.bucket_volume_model_type == DynamicJoinPcl_BucketVolumeModelType_SinglePlane)
                {
                    if(v_rsc.estimate(pcl_scan_partition[bucket], laser_center))
                    {
                        BucketVolume_MaxDistance<Point_In, Point_Out> v_max(bucket1, bucket2, config);
                        v_max.estimate(pcl_scan_partition[bucket], laser_center);
                        double v = v_rsc.getVolume(laser_center);
                        if(v >= v_min.getVolume(laser_center) && v <= v_max.getVolume(laser_center))
                        {
                            //ROS_INFO("plane: <%f, %f, %f, %f>", v_rsc.plane.values[0], v_rsc.plane.values[1], v_rsc.plane.values[2], v_rsc.plane.values[3]);
                            pv = &v_rsc;
                        }
                        else
                        {
                            //ROS_WARN("plane(bad volume): <%f, %f, %f, %f>, dist: %f", v_rsc.plane.values[0], v_rsc.plane.values[1], v_rsc.plane.values[2], v_rsc.plane.values[3], v_min.distance);
                        }
                    }
                    else
                    {
                        //ROS_INFO("plane estimation failed. using min dist with dist=%f", v_min.distance);
                    }
                }
                Quad quad;
                pv->getQuad(laser_center, quad);
                colorizePointInBucket1(quad.color, bucket1, bucket2, config.num_subdivisions);
                quads.push_back(quad);
            }

            // add points from map that do not fall into deletion volume
            for(size_t j = 0; j < pcl_map_partition[bucket].size(); j++)
            {
                double d = dist(laser_center, pcl_map_partition[bucket][j]);

                if(pcl_scan_partition[bucket].size() < 4 ||
                        !pv->testPointForRemoval(pcl_map_partition[bucket][j], laser_center, 0.05 + d * 0.1))
                {
                    if(config.colorize_points)
                        colorizePointInBucket1(pcl_map_partition[bucket][j], bucket1, bucket2, config.num_subdivisions);

                    pcl_map_new.push_back(pcl_map_partition[bucket][j]);
                }
            }

            // add all points from scan
            for(size_t j = 0; j < pcl_scan_partition[bucket].size(); j++)
            {
                Point_Out p;
                copyPoint(pcl_scan_partition[bucket][j], p);

                if(config.colorize_points)
                    colorizePointInBucket1(p, bucket1, bucket2, config.num_subdivisions);

                pcl_map_new.push_back(p);
            }
        }
    }

    // re-add far points
    for(size_t i = 0; i < pcl_map_far.size(); i++)
    {
        pcl_map_new.push_back(pcl_map_far[i]);
    }

    PointCloud_Out temp;
    temp.header = pcl_map_new.header;

    // downsample
    pcl::VoxelGrid<Point_Out> sor;
    sor.setInputCloud(pcl_map_new.makeShared());
    sor.setLeafSize(config.leaf_size, config.leaf_size, config.leaf_size);
    sor.filter(temp);

    pcl_map_new.swap(temp);
}

void op(std::set<point>& a, std::set<point>& b, std::set<point>& c)
{
    set_union(a.begin(), a.end(), b.begin(), b.end(), std::inserter(c, c.begin()));
}

int main(int argc, char **argv)
{
    int i = parse_args(argc, argv);

    pcl::PointCloud<pcl::PointXYZ> tmp;
    std::vector<std::set<point> > pcl;

    for(int j = i, k = 0; j < (argc - 1); j++, k++)
    {
        pcl.resize(pcl.size() + 1);
        tmp.clear();
        load_pcd(argv[j], tmp);
        pcl_to_set(tmp, pcl[k]);
    }

    std::cout << "loaded " << pcl.size() << " point clouds" << std::endl;

    while(pcl.size() > 1)
    {
        int l2 = (pcl.size() + 1) / 2;
        std::cout << "pcl size = " << pcl.size() << " will become " << l2 << std::endl;

        for(int j = 0; j < l2; j++)
        {
            std::set<point> c;
            if((2*j+1) < pcl.size())
            {
                op(pcl[2*j], pcl[2*j+1], c);
                std::cout << "computed union of " << (2*j) << " and " << (2*j+1) << std::endl;
            }
            else
            {
                pcl[2*j].swap(c);
                std::cout << "copy " << (2*j) << " to next level" << std::endl;
            }
            pcl[j].swap(c);
        }
        pcl.resize(l2);
        std::cout << "------------------------------" << std::endl;
    }

    tmp.clear();
    set_to_pcl(pcl[0], tmp);
    save_pcd(argv[argc - 1], tmp);
}

