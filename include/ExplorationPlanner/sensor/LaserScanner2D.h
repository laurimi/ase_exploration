#ifndef LASERSCANNER2D_H
#define LASERSCANNER2D_H

#include "Raytracer2D.h"
#include "PlanarObservation.h"
#include "ExplorationPlanner/grid/PlanarGridOccupancyMap.h"
#include "ExplorationPlanner/grid/PlanarGridBinaryMap.h"
#include "ExplorationPlanner/grid/PlanarGridIndex.h"
#include "ExplorationPlanner/kinematics/PlanarPose.h"
#include "ExplorationPlanner/utils/RNG.h"

#include <string>
#include <boost/random/uniform_int_distribution.hpp>

/**
  * class LaserScanner2D
  *
  */

class LaserScanner2D
{
public:

    // Constructors/Destructors
    //
    LaserScanner2D(double start_angle_rads, double stop_angle_rads,
                   double angle_step_rads, double max_distance_meters,
                   double p_false_pos, double p_false_neg);

    /**
       * @return PlanarObservation
       * @param  pose
       * @param  map_sample_
       * @param  map_prior_
       */
    PlanarObservation sampleMeasurement(const PlanarPose& pose,
                                        const PlanarGridOccupancyMap& map_sample,
                                        const PlanarGridOccupancyMap &map_prior,
                                        const PlanarGridBinaryMap& map_ground_truth,
                                        RNG &rng);

    std::string Print() const;

private:

    // Private attributes
    //
    double starting_angle_rads_;
    double stopping_angle_rads_;
    double angle_step_rads_;
    double max_distance_meters_;
    Raytracer2D raytracer_;

    int8_t prob_false_positive_;
    int8_t prob_false_negative_;
    // Alternative representation via log odds for inverse sensor model
    double ism_free_;
    double ism_occupied_;

    // store incidence angles
    std::vector<double> av_;


    // For sampling cell occupancies and observations
    typedef boost::random::uniform_int_distribution<int8_t> UniDist;

    // Private methods
    //
    bool sampleCellMeasurement(const PlanarGridIndex& idx,
                               const PlanarGridOccupancyMap& map_sample,
                               const PlanarGridOccupancyMap &map_prior,
                               const PlanarGridBinaryMap& map_ground_truth,
                               RNG &rng) const;

    static double prob2lo(int8_t p);
};

#endif // LASERSCANNER2D_H
