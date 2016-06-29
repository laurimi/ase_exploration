#include "ExplorationPlanner/trajectory/TrajectoryChecker.h"
#include <algorithm>

bool TrajectoryChecker::isPoseValid(const PlanarPose &pose) const
{
    PlanarGridIndex idx(0,0);
    invalid_cells_->worldToMap(pose.getX(), pose.getY(), idx);
    return isIndexValid(idx);
}

bool TrajectoryChecker::isTrajectoryValid(const std::vector<PlanarPose> &poses) const
{
    for (auto p : poses)
    {
        if (!isPoseValid(p))
            return false;
    }
    return true;
}

bool TrajectoryChecker::isIndexValid(const PlanarGridIndex& idx) const
{
    if ( !invalid_cells_->isInContainer(idx) ) // cell is unknown
        return allow_unknown_cells_;

    return !(invalid_cells_->getGridValue(idx));
}

unsigned int TrajectoryChecker::getNumOfCellsWithinDistance(double distance_meters) const
{
    if (distance_meters <= 0.0)
        return 1;

    return std::ceil( distance_meters / invalid_cells_->getMetersPerCell() );
}
