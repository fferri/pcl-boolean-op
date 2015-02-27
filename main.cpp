#include "pcl_common.h"

int main(int argc, char **argv)
{
    parse_args(argc, argv);

    pcl::PointCloud<pcl::PointXYZ> tmp;
    std::set<point> a, b, c;
    std::string namea, nameb, namec;

    bool first_operand = true;

    namea = input_filenames[0];
    load_pcd(namea.c_str(), tmp);
    pcl_to_set(tmp, a);

    for(int i = 1; i < input_filenames.size(); i++)
    {
        if(!first_operand)
        {
            c.swap(a);
            namea = namec;
        }

        b.clear();
        c.clear();
        tmp.clear();

        nameb = input_filenames[i];
        load_pcd(nameb.c_str(), tmp);
        pcl_to_set(tmp, b);

        op(a, b, c);

        std::stringstream namec_ss;
        namec_ss << "pcl_" << i;
        namec = namec_ss.str();

        if(verbose)
            std::cout << namea << " * " << nameb << " = " << namec << " (" << c.size() << " points)" << std::endl;

        first_operand = false;
    }

    if(verbose)
        std::cout << "result is " << namec << std::endl;

    tmp.clear();
    set_to_pcl(c, tmp);
    save_pcd(output_filename.c_str(), tmp);
}
