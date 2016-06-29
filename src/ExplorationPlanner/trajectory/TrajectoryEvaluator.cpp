#include "ExplorationPlanner/trajectory/TrajectoryEvaluator.h"


TrajectoryValue TrajectoryEvaluator::evaluateTrajectory(const std::vector<PlanarPose>& trajectory,
                                   const PlanarGridOccupancyMap& map,
                                   RNG& rng)
{
    if ( trajectory.size() == 0)
        return TrajectoryValue();

    // start out with an empty map; we will copy elements to it from input map when necessary.
    PlanarGridOccupancyMap map_sample(map.getOrigin(), map.getMetersPerCell());

    // we shall assume an empty ground truth map for this case.
    PlanarGridBinaryMap gt(map.getOrigin(), map.getMetersPerCell());

    TrajectoryValue tv;
    for (auto it = trajectory.begin(), iend = trajectory.end(); it != iend; ++it)
    {
        PlanarObservation observation = sensors_[omp_get_thread_num()].sampleMeasurement(*it, map_sample, map, gt, rng);
        // copy all elements from input map to map_sample that are not there yet.
        map_sample.copyCellsFromOtherMap(map, observation);

        // compute prior and posterior entropy
        double prior_entropy = map_sample.getEntropyInCells(observation);
        map_sample.update(observation);
        double posterior_entropy = map_sample.getEntropyInCells(observation);

        double mutual_information = (prior_entropy - posterior_entropy);
        tv.insert(mutual_information, observation);
    }

    return tv;
}
