#ifndef IPLANARGRIDMAP_H
#define IPLANARGRIDMAP_H

#include "PlanarGridIndex.h"
#include "ExplorationPlanner/kinematics/PlanarPose.h"

/**
  * class IPlanarGridMap
  *
  */

/******************************* Abstract Class ****************************
IPlanarGridMap does not have any pure virtual methods, but its author
  defined it as an abstract class, so you should not use it directly.
  Inherit from it instead and create only objects from the derived classes
*****************************************************************************/

class IPlanarGridMap
{
public:

    // Constructors/Destructors
    //
    IPlanarGridMap(PlanarPose origin, double meters_per_cell)
        : origin_(origin), meters_per_cell_(meters_per_cell)
    {

    }

    virtual ~IPlanarGridMap() {}

    /**
   * Get the value of origin_
   * @return the value of origin_
   */
    const PlanarPose& getOrigin() const  {
        return origin_;
    }

    double getMetersPerCell() const  {
        return meters_per_cell_;
    }

    /**
   * @param  wx world x-coordinate to convert to index
   * @param  wy world y-coordinate to convert to index
   * @param  idx assigned to index value at (wx,wy)
   */
    void worldToMap(double wx, double wy, PlanarGridIndex& idx) const
    {
        idx.setX( static_cast<int> ((wx - origin_.getX()) / meters_per_cell_) );
        idx.setY( static_cast<int> ((wy - origin_.getY()) / meters_per_cell_) );
    }


    /**
   * @param  idx index of cell *center* to convert to world coordinates
   * @param  wx assigned to x-coordinate at index cell's *center point*
   * @param  wy assigned to y-coordinate at index cell's *center point*
   */
    void mapToWorld(PlanarGridIndex idx, double& wx, double& wy) const
    {
        wx = origin_.getX() + (idx.getX() + 0.5) * meters_per_cell_;
        wy = origin_.getY() + (idx.getY() + 0.5) * meters_per_cell_;
    }

private:

    // Private attributes
    //
    PlanarPose origin_; // world coordinate for the corner of cell (0,0)
    double meters_per_cell_; // resolution
};

#endif // IPLANARGRIDMAP_H
