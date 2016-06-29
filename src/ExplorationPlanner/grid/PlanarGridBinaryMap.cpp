#include "ExplorationPlanner/grid/PlanarGridBinaryMap.h"
#include <boost/unordered_set.hpp>

void PlanarGridBinaryMap::dilate()
{
    boost::unordered_set<PlanarGridIndex> listtrue;
    for ( ContainerConstIter it = begin(), iend = end(); it != iend; ++it)
    {
        if (it->second)
        {
            PlanarGridIndex idx = it->first;
            for (int w = -1; w <= 1; ++w)
                for (int h = -1; h <= 1; ++h)
                {
                    if (w == 0 && h == 0)
                        continue;
                    PlanarGridIndex addidx(idx.getX()+w, idx.getY()+h);
                    if ( isInContainer(addidx) ) // don't add any new items to the map
                        listtrue.insert(addidx);
                }
        }
    }

    // Now mark all the cells as "true"
    for( boost::unordered_set<PlanarGridIndex>::const_iterator it = listtrue.begin(), iend = listtrue.end(); it != iend; ++it)
        setGridValue(*it, true);
}
