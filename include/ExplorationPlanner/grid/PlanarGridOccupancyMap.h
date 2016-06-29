#ifndef PLANARGRIDOCCUPANCYMAP_H
#define PLANARGRIDOCCUPANCYMAP_H
#include "ExplorationPlanner/grid/IPlanarGridMap.h"
#include "ExplorationPlanner/grid/PlanarGridContainer.h"
#include "ExplorationPlanner/sensor/PlanarObservation.h"
#include "ExplorationPlanner/utils/EntropyLUT.h"
#include "ExplorationPlanner/utils/LogOddsLUT.h"

#include <stdint.h> // int8_t

/**
  * class PlanarGridOccupancyMap
  *
  */
class PlanarGridOccupancyMap : virtual public IPlanarGridMap, virtual public PlanarGridContainer<int8_t>
{
public:
    // Constructors/Destructors
    //
    PlanarGridOccupancyMap(PlanarPose origin, double meters_per_cell);

    void update(const PlanarObservation& cells);

    void copyCellsFromOtherMap(const PlanarGridOccupancyMap& other_map, const PlanarObservation& cells);

    /**
   * @return double
   */
    double getEntropy() const;

    /**
   * @return double
   * @param  cells
   * @param  prior_map
   */
    double getEntropyInCells(const PlanarObservation& cells) const;

private:
    void updateCell(const PlanarGridIndex& idx, const PlanarCellObservation& obs);
    EntropyLUT entropy_lut_;
    LogOddsLUT logodds_lut_;
    static int8_t lo2prob(double l);
};

#endif // PLANARGRIDOCCUPANCYMAP_H
