#include "ExplorationPlanner/grid/PlanarGridOccupancyMap.h"

// Constructors/Destructors
//  
PlanarGridOccupancyMap::PlanarGridOccupancyMap(PlanarPose origin, double meters_per_cell)
    : IPlanarGridMap(origin, meters_per_cell)
{

}

//  
// Methods
//  

void PlanarGridOccupancyMap::update(const PlanarObservation& cells)
{
    for(PlanarObservation::ContainerConstIter it = cells.begin(), iend = cells.end(); it != iend; ++it)
    {
        updateCell(it->first, it->second);
    }
}

void PlanarGridOccupancyMap::copyCellsFromOtherMap(const PlanarGridOccupancyMap& other_map, const PlanarObservation& cells)
{
    for(PlanarObservation::ContainerConstIter it = cells.begin(), iend = cells.end(); it != iend; ++it)
    {
        ContainerConstIter this_map_it = find( it->first );
        if ( this_map_it == end() ) // Cell is not in map, try to add it
        {
            // we can only copy from the other map if the cell is present there
            ContainerConstIter other_map_it = other_map.find(it->first);
            if ( other_map_it != other_map.end() )
            {
                setGridValue(it->first, other_map_it->second); // set value for this
            }
        }
    }
}

void PlanarGridOccupancyMap::updateCell(const PlanarGridIndex& idx, const PlanarCellObservation &obs)
{
    ContainerConstIter map_it = find( idx );
    if ( map_it != end() ) // Cell is in map.
    {
        // log odds representation of (occupancy prob) + (inverse sensor model)
        const int8_t new_p = lo2prob( logodds_lut_.get(map_it->second) + obs.getISMValue() );
        setGridValue(idx, new_p);
    }
    else
    { // New cell needs to be inserted
        setGridValue(idx, lo2prob(obs.getISMValue()) );
    }
}

double PlanarGridOccupancyMap::getEntropy() const
{
    double e = 0.0;

    for (ContainerConstIter it = begin(), iend = end(); it != iend; ++it)
        e += entropy_lut_.get(it->second);
    return e;
}


double PlanarGridOccupancyMap::getEntropyInCells(const PlanarObservation& cells) const
{
    // Compute entropy in specified cells: Use map sample, or if not availabe, use prior map.
    double e = 0.0;
    for(PlanarObservation::ContainerConstIter it = cells.begin(), iend = cells.end(); it != iend; ++it)
    {
        ContainerConstIter map_it = find(it->first);
        if ( map_it != end() )
            e += entropy_lut_.get(map_it->second);
        else
            e += 1.0; // Unknown cell, entropy of 1 bit
    }
    return e;
}

int8_t PlanarGridOccupancyMap::lo2prob(double l)
{
    return static_cast<int8_t>( 100 * (1 - 1 / (1 + exp(l)))  );
}
