#ifndef SMCPLANNERPARAMETERS_H
#define SMCPLANNERPARAMETERS_H
#include "ICoolingSchedule.h"
#include "ExplorationPlanner/kinematics/IPlanarRobotVelCmdSampler.h"
#include <vector>
#include <memory>
#include <string>

class SMCPlannerParameters
{
public:
    SMCPlannerParameters(const ICoolingSchedule& schedule,
                         unsigned int num_particles,
                         double resampling_threshold_fraction);


    std::unique_ptr<SMCPlannerParameters> clone() const
    {
        std::unique_ptr<SMCPlannerParameters> s( new SMCPlannerParameters(*cooling_schedule_,
                                                                          num_particles_,
                                                                          resampling_threshold_fraction_) );
        for ( SamplerPtrVec::const_iterator it = kernels_.begin(), iend = kernels_.end(); it != iend; ++it)
            s->addKernel( **it );
        return s;
    }


    unsigned int getNumberOfParticles() const
    {
        return num_particles_;
    }

    unsigned int getNumberOfIterations() const
    {
        return kernels_.size();
    }

    unsigned int getNumberOfReplicates(unsigned int iteration) const
    {
        return cooling_schedule_->getNumberOfReplicates(iteration);
    }

    IPlanarRobotVelCmdSampler& getKernel(unsigned int k) const
    {
        if (k > kernels_.size())
            throw std::out_of_range("getKernel: Kernel requested out of range");
        return *kernels_[k];
    }

    double getResamplingThreshold() const
    {
        return resampling_threshold_fraction_;
    }

    void addKernel(const IPlanarRobotVelCmdSampler& k);


    std::string Print() const;

private:
    typedef std::vector<std::unique_ptr<IPlanarRobotVelCmdSampler> > SamplerPtrVec;

    SamplerPtrVec kernels_;
    std::unique_ptr<ICoolingSchedule> cooling_schedule_;
    unsigned int num_particles_;
    double resampling_threshold_fraction_;
};

#endif // SMCPLANNERPARAMETERS_H
