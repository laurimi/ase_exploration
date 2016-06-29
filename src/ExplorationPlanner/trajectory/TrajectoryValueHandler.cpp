#include "ExplorationPlanner/trajectory/TrajectoryValueHandler.h"
#include <numeric>

double TrajectoryValueHandler::getWeightFactor(double subtract, double scaleFactor) const
{
    // The scaler should force sums of discounted rewards to the range of 0 ... 1.
    // --> Log arguments are between 1 and 2
    double w = 0.0;
    for (auto v : values_)
        w += log( 1 + ((v.getSumOfDiscountedRewards(discount_) - subtract) / scaleFactor) );

    return exp(w);
}

double TrajectoryValueHandler::getMinSumOfDiscRew() const
{
    double min = std::numeric_limits<double>::max();
    for (auto v : values_)
        min = std::min(min, v.getSumOfDiscountedRewards(discount_));
    return min;
}

double TrajectoryValueHandler::getMaxSumOfDiscRew() const
{
    double max = -std::numeric_limits<double>::max();
    for (auto v : values_)
        max = std::max(max, v.getSumOfDiscountedRewards(discount_));
    return max;
}

double TrajectoryValueHandler::getAvgSumOfDiscRew() const
{
    double avg = 0.0;
    int i = 0;
    for (auto v : values_)
        avg += (v.getSumOfDiscountedRewards(discount_) - avg) / ( ++i );
    return avg;
}
