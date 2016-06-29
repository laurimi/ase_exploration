#ifndef TRAJECTORYVALUEHANDLER_H
#define TRAJECTORYVALUEHANDLER_H
#include "TrajectoryValue.h"
#include <vector>
#include <memory>

class TrajectoryValueHandler
{
public:
    TrajectoryValueHandler(double discount)
        : discount_(discount)
    {

    }

    void addValue(const TrajectoryValue& v)
    {
        values_.push_back( v );
    }

    // Returns weight factor for the particle:
    //    new weight = old weight * weight factor
    double getWeightFactor(double subtract, double scaleFactor) const;
    double getMinSumOfDiscRew() const;
    double getMaxSumOfDiscRew() const;
    double getAvgSumOfDiscRew() const;

private:
    double discount_;
    std::vector<TrajectoryValue> values_;
};

#endif /* TRAJECTORYVALUEHANDLER_H */
