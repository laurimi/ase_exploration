#ifndef RANGECONSTRAINT_H
#define RANGECONSTRAINT_H
#include "IConstraint.h"
/**
  * class RangeConstraint
  *
  */

class RangeConstraint : virtual public IConstraint
{
public:

    // Constructors/Destructors
    //
    RangeConstraint(double rmin, double rmax)
        : range_min_(rmin), range_max_(rmax)
    {

    }

    virtual std::unique_ptr<IConstraint> clone() const;

    /**
   * @return bool
   * @param  value
   */
    bool check(double value) const;

    double getMinValue() const {
        return range_min_;
    }
    double getMaxValue() const {
        return range_max_;
    }
    double forceToNearestLimit(double value) const
    {
        if (value < range_min_)
            return range_min_;
        else if ( value > range_max_)
            return range_max_;
        else
            return value;
    }

private:

    // Private attributes
    //
    double range_min_;
    double range_max_;
};

#endif // RANGECONSTRAINT_H
