#ifndef PLANARROBOTVELCMDSAMPLERINDEPENDENTGAUSSIAN_H
#define PLANARROBOTVELCMDSAMPLERINDEPENDENTGAUSSIAN_H
#include "VelocityPlanarKinematics.h"
#include "IPlanarRobotVelCmdSampler.h"
#include "ExplorationPlanner/utils/RNG.h"
#include <boost/random/normal_distribution.hpp>

/**
  * class PlanarRobotVelCmdSamplerIndependentGaussian
  *
  */

class PlanarRobotVelCmdSamplerIndependentGaussian : virtual public IPlanarRobotVelCmdSampler
{
public:
    // Constructors/Destructors
    //
    PlanarRobotVelCmdSamplerIndependentGaussian(
            double linear_velocity_stdev,
            double angular_velocity_stdev,
            double final_rotation_stdev,
            double default_duration_secs = 1.0);

    virtual std::unique_ptr<IPlanarRobotVelCmdSampler> clone() const
    {
        return std::unique_ptr<IPlanarRobotVelCmdSampler>( new PlanarRobotVelCmdSamplerIndependentGaussian(lv_std_,
                                                                                                           av_std_,
                                                                                                           fr_std_,
                                                                                                           default_ctrl_duration_secs_) );
    }

    virtual PlanarRobotVelCmd sample(const PlanarRobotVelCmd& old_cmd,
                                     const IPlanarKinematics& kinematics,
                                     RNG &rng);

    std::string Print() const;

private:
    typedef boost::random::normal_distribution<double> NormalDist;
    double default_ctrl_duration_secs_;
    double lv_std_;
    double av_std_;
    double fr_std_;
};

#endif // PLANARROBOTVELCMDSAMPLERINDEPENDENTGAUSSIAN_H
