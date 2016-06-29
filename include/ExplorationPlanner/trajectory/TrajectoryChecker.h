#ifndef TRAJECTORYCHECKER_H
#define TRAJECTORYCHECKER_H
#include "ExplorationPlanner/grid/PlanarGridBinaryMap.h"
#include "ExplorationPlanner/kinematics/PlanarPose.h"
#include <boost/shared_ptr.hpp>
#include <vector>

class TrajectoryChecker
{
public:
    TrajectoryChecker(const boost::shared_ptr<PlanarGridBinaryMap>& invalid_cells,
                      bool allow_unknown_cells)
        : invalid_cells_(invalid_cells),
          allow_unknown_cells_(allow_unknown_cells)
    {
    }

    bool isPoseValid(const PlanarPose& pose) const;
    bool isTrajectoryValid(const std::vector<PlanarPose>& poses) const;
    bool isIndexValid(const PlanarGridIndex& idx) const;
    unsigned int getNumOfCellsWithinDistance(double distance_meters) const;

private:
    boost::shared_ptr<PlanarGridBinaryMap> invalid_cells_;
    bool allow_unknown_cells_;
};

#endif /* TRAJECTORYCHECKER_H */
