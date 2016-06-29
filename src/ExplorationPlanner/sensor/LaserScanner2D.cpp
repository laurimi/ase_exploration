#include "ExplorationPlanner/sensor/LaserScanner2D.h"
#include <sstream>

// Constructors/Destructors
//  
LaserScanner2D::LaserScanner2D(double start_angle_rads, double stop_angle_rads,
                               double angle_step_rads, double max_distance_meters,
                               double p_false_pos, double p_false_neg)
    : starting_angle_rads_(start_angle_rads),
      stopping_angle_rads_(stop_angle_rads),
      angle_step_rads_(angle_step_rads),
      max_distance_meters_(max_distance_meters),
      prob_false_positive_( static_cast<int8_t>(p_false_pos*100)),
      prob_false_negative_( static_cast<int8_t>(p_false_neg*100)),
      ism_free_( prob2lo(prob_false_positive_) ),
      ism_occupied_( prob2lo(100 - prob_false_negative_) )
{
    // Populate list of theoretical beam endpoints from origin
    for ( double a = starting_angle_rads_; a < stopping_angle_rads_; a+=angle_step_rads_)
         av_.push_back( a );
}

//  
// Methods
//
PlanarObservation LaserScanner2D::sampleMeasurement(const PlanarPose &pose,
                                                    const PlanarGridOccupancyMap& map_sample,
                                                    const PlanarGridOccupancyMap& map_prior,
                                                    const PlanarGridBinaryMap& map_ground_truth,
                                                    RNG& rng)
{
    PlanarObservation obs;
    PlanarGridIndex start(0,0);
    map_sample.worldToMap(pose.getX(), pose.getY(), start);

    for ( auto a : av_ )
    {
        // theoretical end point of the beam
        const double angle = pose.getTheta() + a;
        const double x_end = pose.getX() + max_distance_meters_ * cos(angle);
        const double y_end = pose.getY() + max_distance_meters_ * sin(angle);
        PlanarGridIndex stop(0,0);
        map_sample.worldToMap(x_end, y_end, stop);

        raytracer_.initialize(start.getX(), start.getY(), stop.getX(), stop.getY());
        // Get next raytraced grid cell until the beam terminates or hits an obstacle
        int x, y;
        while ( raytracer_.getNext(x, y) )
        {
            const PlanarGridIndex r(x, y);
            bool o = false; // the observation perceived
            if (!obs.isInContainer(r))
            { // Observation does not include index; draw a sample
                o = sampleCellMeasurement(r, map_sample, map_prior, map_ground_truth, rng);
                obs.setGridValue(r, PlanarCellObservation(o, (o ? ism_occupied_ : ism_free_) ) );
            }
            else
            { // Observation includes the index; get its value
                o = obs.find(r)->second.getObservation();
            }

            if (o)
                break; // beam hit obstacle
        }

    }
    return obs;
}

bool LaserScanner2D::sampleCellMeasurement(const PlanarGridIndex& idx,
                                           const PlanarGridOccupancyMap& map_sample,
                                           const PlanarGridOccupancyMap& map_prior,
                                           const PlanarGridBinaryMap& map_ground_truth,
                                           RNG& rng) const
{
    // First try to draw sample from ground truth map, if it is not possible, draw from current map sample,
    // then if that can't be done, from current map_prior, and if even that is not possible,
    // assume the cell free (unknown).
    bool cellOccupancy = false;
    if (  !map_ground_truth.empty() &&  map_ground_truth.isInContainer(idx) )
    {
        cellOccupancy = map_ground_truth.getGridValue(idx);
    }
    else
    {
        // try to sample an occupancy value for the cell according to current occupancy probability
        // priority: current map sample
        // backup: current map prior
        int8_t cellOccupancyProb = 0;
        if ( !map_sample.empty() && map_sample.isInContainer(idx) )
        {
            cellOccupancyProb = map_sample.getGridValue(idx);
        }
        else if ( !map_prior.empty() && map_prior.isInContainer(idx) )
        {
            cellOccupancyProb = map_prior.getGridValue(idx);
        }
        // sampling
        boost::random::uniform_int_distribution<int8_t> dc(0,100);
        cellOccupancy = (cellOccupancyProb >= rng(dc));
    }
    // If neither condition triggers, we assume that the cell is free (do nothing)

    // Sample an observation of the cell given its occupancy value and sensor properties
    boost::random::uniform_int_distribution<int8_t> dc(0,100);
    const int8_t r = rng(dc);
    bool obs(cellOccupancy ? ( r >= prob_false_negative_ ) : ( r <= prob_false_positive_ ) );
    return obs;
}


std::string LaserScanner2D::Print() const
{
    std::stringstream ss;
    ss << "LaserScanner2D: \n" <<
          "   start angle  : " << starting_angle_rads_ << " rad\n" <<
          "   stop angle   : " << stopping_angle_rads_ << " rad\n" <<
          "   angle step   : " << angle_step_rads_ << " rad\n" <<
          "   max distance : " << max_distance_meters_ << " m\n" <<
          "   P(false pos.): " << static_cast<int>(prob_false_positive_) << "\n" <<
          "   P(false neg.): " << static_cast<int>(prob_false_negative_) << "\n" <<
          "   ISM(occupied): " << ism_occupied_ << "\n" <<
          "   ISM(free)    : " << ism_free_ << "\n" <<
          "   Cache size   : " << raytracer_.cacheSize();

    return ss.str();
}

double LaserScanner2D::prob2lo(int8_t p)
{
    double pr = static_cast<double>(p)/100;
    return log( pr / (1-pr));
}
