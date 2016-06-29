#ifndef TRAJECTORYEVALUATOR_H
#define TRAJECTORYEVALUATOR_H
#include "TrajectoryValue.h"
#include "ExplorationPlanner/kinematics/PlanarPose.h"
#include "ExplorationPlanner/planner/ExplorationTaskState.h"
#include "ExplorationPlanner/sensor/LaserScanner2D.h"
#include "ExplorationPlanner/utils/RNG.h"
#include <vector>
#include <omp.h>

class TrajectoryEvaluator
{
public:
    virtual ~TrajectoryEvaluator() {}
    TrajectoryEvaluator(const LaserScanner2D& sensor)
        : sensors_(std::max(1, omp_get_max_threads()), sensor)
    {

    }

    TrajectoryValue evaluateTrajectory(const std::vector<PlanarPose>& trajectory,
                  const PlanarGridOccupancyMap &map,
                  RNG& rng);

private:
    std::vector<LaserScanner2D> sensors_; // each thread has own sensor object
};

#endif /* TRAJECTORYEVALUATOR_H */
