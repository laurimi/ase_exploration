#ifndef PLANARGRIDBINARYMAP_H
#define PLANARGRIDBINARYMAP_H
#include "IPlanarGridMap.h"
#include "PlanarGridContainer.h"

/**
  * class PlanarGridOccupancyMap
  *
  */
class PlanarGridBinaryMap : virtual public IPlanarGridMap, virtual public PlanarGridContainer<bool>
{
public:
    PlanarGridBinaryMap(PlanarPose origin, double meters_per_cell)
        : IPlanarGridMap(origin, meters_per_cell),
          PlanarGridContainer<bool>()
    {
    }

    // increase "safety margin" by marking one pixel around each cell with value "true"
    void dilate();
};

#endif // PLANARGRIDBINARYMAP_H
