#include "ExplorationPlanner/planner/SMCPlannerParameters.h"
#include <iostream>
#include <sstream>

SMCPlannerParameters::SMCPlannerParameters(const ICoolingSchedule &schedule,
                                           unsigned int num_particles,
                                           double resampling_threshold_fraction)
    : cooling_schedule_(schedule.clone()),
      num_particles_(num_particles),
      resampling_threshold_fraction_(resampling_threshold_fraction)
{

}

void SMCPlannerParameters::addKernel(const IPlanarRobotVelCmdSampler &k)
{
    kernels_.emplace_back(k.clone());
}


std::string SMCPlannerParameters::Print() const
{
    std::stringstream ss;
    ss << "::SMC Planner parameters::\n";
    ss << "-Cooling schedule-\n" << cooling_schedule_->Print() << "\n- Kernels -\nNumber of kernels: " << kernels_.size() << "\n";
    for (unsigned int i=0; i < kernels_.size(); ++i)
    {
        ss << "Kernel " << i << ":\n";
        ss << kernels_[i]->Print() << "\n";
    }
    return ss.str();
}

// Tells how to print SMCPlannerParameters
std::ostream &operator<<(std::ostream &os, const SMCPlannerParameters& s)
{
    os << s.Print();
    return os;

    return os;
}
