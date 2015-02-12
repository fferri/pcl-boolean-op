#include "pcl_common.h"

void op(std::set<point>& a, std::set<point>& b, std::set<point>& c)
{
    set_union(a.begin(), a.end(), b.begin(), b.end(), std::inserter(c, c.begin()));
}

