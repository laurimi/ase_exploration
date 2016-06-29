#ifndef LINEARCOOLINGSCHEDULE_H
#define LINEARCOOLINGSCHEDULE_H
#include "ICoolingSchedule.h"
#include <sstream>

// The linear cooling schedule returns the number of
// replications as C = a*iteration + b

class LinearCoolingSchedule : virtual public ICoolingSchedule
{
public:
    LinearCoolingSchedule(unsigned int a, unsigned int b)
        : a_(a), b_(b)
    {
    }

    unsigned int getNumberOfReplicates(unsigned int iteration)
    {
        return a_*iteration + b_;
    }

    virtual std::unique_ptr<ICoolingSchedule> clone() const
    {
        return std::unique_ptr<ICoolingSchedule>(new LinearCoolingSchedule(a_, b_) );
    }

    std::string Print() const
    {
        std::stringstream ss;
        ss << "Linear schedule: C = a*x + b\na = " << a_ << ", b = " << b_;
        return ss.str();
    }

private:
    unsigned int a_;
    unsigned int b_;

};

#endif // LINEARCOOLINGSCHEDULE_H
