#ifndef PLANAROBSERVATION_H
#define PLANAROBSERVATION_H
#include "ExplorationPlanner/grid/PlanarGridContainer.h"
#include "ExplorationPlanner/grid/PlanarGridIndex.h"
#include "ExplorationPlanner/sensor/PlanarCellObservation.h"
#include <boost/unordered_map.hpp>
/**
  * class PlanarObservation
  *
  */

class PlanarObservation : virtual public PlanarGridContainer<PlanarCellObservation>
{

};

#endif // PLANAROBSERVATION_H
