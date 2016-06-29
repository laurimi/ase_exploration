#ifndef ICOOLINGSCHEDULE_H
#define ICOOLINGSCHEDULE_H
#include <string>
#include <memory>

class ICoolingSchedule
{
public:
    virtual unsigned int getNumberOfReplicates(unsigned int iteration) = 0;
    virtual std::unique_ptr<ICoolingSchedule> clone() const = 0;
    virtual std::string Print() const = 0;
};



#endif // ICOOLINGSCHEDULE_H
