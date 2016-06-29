#ifndef ICONSTRAINT_H
#define ICONSTRAINT_H
#include <memory>
/**
  * class IConstraint
  *
  */

class IConstraint
{
public:
    virtual ~IConstraint() {}

    virtual std::unique_ptr<IConstraint> clone() const = 0;

    /**
   * @return bool
   * @param  value
   */
    virtual bool check (double value) const = 0;

    virtual double getMinValue() const = 0;
    virtual double getMaxValue() const = 0;

    virtual double forceToNearestLimit(double value) const = 0;
};

#endif // ICONSTRAINT_H
