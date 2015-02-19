#include "pcl_common.h"

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

