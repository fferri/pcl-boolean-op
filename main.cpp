#include "pcl_common.h"

int main(int argc, char **argv)
{
    int i = parse_args(argc, argv);

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
