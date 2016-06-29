#include "ExplorationPlanner/kinematics/RangeConstraint.h"

bool RangeConstraint::check(double value) const
{
    return ( (value >= range_min_ ) && (value <= range_max_) );
}

std::unique_ptr<IConstraint> RangeConstraint::clone() const
{
    return std::unique_ptr<IConstraint>( new RangeConstraint(range_min_, range_max_));
}


