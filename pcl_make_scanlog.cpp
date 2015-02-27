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

void loadPCL(const char *filename, pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Vector4f& t, Eigen::Matrix3f R)
{
        pcl::PCLPointCloud2 tmp;
        Eigen::Quaternionf r;
        if(pcl::io::loadPCDFile(filename, tmp, t, r) == -1)
        {
            std::cerr << "error: load of " << filename << " failed" << std::endl;
            exit(1);
        }
        pcl::fromPCLPointCloud2(tmp, cloud);
        r.normalize();
        R = r.toRotationMatrix();
}

#define IGNORE_ROTATION

int main(int argc, char **argv)
{
    Eigen::Vector4f last_t;
    Eigen::Matrix3f last_R;

    if(argc < 2 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)
        usage(argv[0], 1);

    for(int j = 1; j < argc; j++)
    {
        Eigen::Vector4f t;
        Eigen::Matrix3f R;
        pcl::PointCloud<pcl::PointXYZ> tmp2;

        loadPCL(argv[j], tmp2, t, R);

#ifdef IGNORE_ROTATION
        std::cout << "NODE " << t(0) << " " << t(1) << " " << t(2) << " 0 0 0" << std::endl;
#else
        Eigen::Matrix<float,3,1> euler = R.eulerAngles(2, 1, 0);
        float yaw = euler(0,0);
        float pitch = euler(1,0);
        float roll = euler(2,0);
        std::cout << "NODE " << t(0) << " " << t(1) << " " << t(2) << " " << roll << " " << pitch << " " << yaw << std::endl;
#endif

        for(size_t i = 0; i < tmp2.size(); i++)
        {
            Eigen::Vector3f p, t3;
            p << tmp2[i].x, tmp2[i].y, tmp2[i].z;
            t3 << t.x(), t.y(), t.z();
#ifdef IGNORE_ROTATION
            p = p - t3;
#else
            p = R.inverse() * p - t3;
#endif
            std::cout << p.x() << " " << p.y() << " " << p.z() << std::endl;
        }

        last_t = t;
        last_R = R;
    }
}

