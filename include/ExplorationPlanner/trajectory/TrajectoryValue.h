#ifndef TRAJECTORYVALUE_H
#define TRAJECTORYVALUE_H
#include "ExplorationPlanner/kinematics/PlanarPose.h"
#include "ExplorationPlanner/sensor/PlanarObservation.h"
#include <vector>

class TrajectoryValue
{
public:
    TrajectoryValue() :
        rewards_(),
        observations_()
    {

    }

    double getSumOfDiscountedRewards(double discount) const
    {
        double d = 1.0;
        double discounted_sum = 0.0;
        for (auto r : rewards_)
        {
            discounted_sum += d * r;
            d *= discount;
        }
        return discounted_sum;
    }

    void insert(const double reward)
    {
        rewards_.push_back(reward);
    }

    void insert(const double reward, const PlanarObservation& observation)
    {
        rewards_.push_back(reward);
        observations_.push_back(observation);
    }


private:
    std::vector<double> rewards_;
    std::vector<PlanarObservation> observations_;
};

#endif /* TRAJECTORYVALUE_H */
