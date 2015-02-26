#define op op_intersection
#include "pcl_intersection.cpp"
#undef op

#define op op_union
#include "pcl_union.cpp"
#undef op

void usage2(const char *self)
{
    std::cout << "usage: " << self << " [options] a.pcd b.pcd" << std::endl
        << "options:" << std::endl
        << "  --tolerance <t>" << std::endl
        << "  -t <t>            two points are considered equal if:" << std::endl
        << "                    |x1-x2|<t && |y1-y2|<t && |z1-z2|<t" << std::endl;
}

extern double tolerance;

int main(int argc, char **argv)
{
    int i = 1;

    if(argc < (i + 1))
    {
        usage2(argv[0]);
        exit(1);
    }

    if(strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--tolerance") == 0)
    {
        i++;
        if(argc < (i + 1))
        {
            usage2(argv[0]);
            exit(1);
        }
        tolerance = atof(argv[i++]);
        std::cout << "info: tolerance set to " << tolerance << std::endl;
    }
    else
    {
        std::cout << "info: using default tolerance of " << tolerance << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ> tmp;
    std::set<point> a, b, c_intersection, c_union;

    if(i + 2 != argc)
    {
        usage2(argv[0]);
        exit(1);
    }

    tmp.clear();
    load_pcd(argv[i++], tmp);
    pcl_to_set(tmp, a);

    tmp.clear();
    load_pcd(argv[i++], tmp);
    pcl_to_set(tmp, b);

    op_intersection(a, b, c_intersection);
    op_union(a, b, c_union);

    std::cout << "jaccard similarity index: " << c_intersection.size()/double(c_union.size()) << std::endl;
}
