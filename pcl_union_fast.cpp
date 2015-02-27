#include "pcl_common.h"

void op(std::set<point>& a, std::set<point>& b, std::set<point>& c)
{
    set_union(a.begin(), a.end(), b.begin(), b.end(), std::inserter(c, c.begin()));
}

int main(int argc, char **argv)
{
    parse_args(argc, argv);

    pcl::PointCloud<pcl::PointXYZ> tmp;
    std::vector<std::set<point> > pcl;

    int k = 0;
    pcl.resize(input_filenames.size());
    for(std::vector<std::string>::iterator it = input_filenames.begin(); it != input_filenames.end(); ++it)
    {
        load_pcd(it->c_str(), tmp);
        pcl_to_set(tmp, pcl[k++]);
        tmp.clear();
    }

    if(verbose)
        std::cout << "loaded " << pcl.size() << " point clouds" << std::endl;

    while(pcl.size() > 1)
    {
        int l2 = (pcl.size() + 1) / 2;

        for(int j = 0; j < l2; j++)
        {
            std::set<point> c;
            if((2*j+1) < pcl.size())
            {
                op(pcl[2*j], pcl[2*j+1], c);
            }
            else
            {
                pcl[2*j].swap(c);
            }
            pcl[j].swap(c);
        }
        pcl.resize(l2);
    }

    set_to_pcl(pcl[0], tmp);
    save_pcd(output_filename.c_str(), tmp);
}

