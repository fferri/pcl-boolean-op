#define op op_intersection
#include "pcl_intersection.cpp"
#undef op

#define op op_union
#include "pcl_union.cpp"
#undef op

int main(int argc, char **argv)
{
    int i = parse_args(argc, argv);

    pcl::PointCloud<pcl::PointXYZ> tmp;
    std::set<point> a, b, c_intersection, c_union;

    if(i + 2 >= argc)
    {
        std::cerr << "usage: " << argv[0] << " a.pcd b.pcd" << std::endl;
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
