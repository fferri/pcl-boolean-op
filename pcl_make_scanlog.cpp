#include <cmath>
#include <iostream>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

void usage(const char *self, int exitcode)
{
    std::cout << "usage: " << self << " cloud1.pcd ... cloudN.pcd" << std::endl;
    exit(exitcode);
}

int main(int argc, char **argv)
{
    Eigen::Vector4f last_t;
    Eigen::Matrix3f last_R;

    if(argc < 2 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)
        usage(argv[0], 1);

    for(int j = 1; j < argc; j++)
    {
        pcl::PCLPointCloud2 tmp;
        Eigen::Vector4f t;
        Eigen::Quaternionf r;
        if(pcl::io::loadPCDFile(argv[j], tmp, t, r) == -1)
        {
            std::cerr << "error: load of " << argv[j] << " failed" << std::endl;
            exit(1);
        }
        r.normalize();
        Eigen::Matrix3f R = r.toRotationMatrix();

        pcl::PointCloud<pcl::PointXYZ> tmp2;
        pcl::fromPCLPointCloud2(tmp, tmp2);

        Eigen::Vector4f t_;
        Eigen::Matrix3f R_ = R;
        if(j > 1)
        {
            t_ = t - last_t;
            R_ = last_R.inverse() * R;
        }
        Eigen::Matrix<float,3,1> euler = R_.eulerAngles(2, 1, 0);
        float yaw = euler(0,0);
        float pitch = euler(1,0);
        float roll = euler(2,0);
        std::cout << "NODE "
            << t_(0) << " " << t_(1) << " " << t_(2) << " "
            //<< roll << " " << pitch << " " << yaw
            << "0 0 0"
            << std::endl;

        for(size_t i = 0; i < tmp2.size(); i++)
            std::cout << tmp2[i].x << " " << tmp2[i].y << " " << tmp2[i].z << std::endl;

        last_t = t;
        last_R = R;
    }
}

