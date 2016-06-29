#ifndef PLANARROBOTVELCMDSAMPLERUNIFORM_H
#define PLANARROBOTVELCMDSAMPLERUNIFORM_H
#include "IPlanarRobotVelCmdSampler.h"
#include "VelocityPlanarKinematics.h"
#include "ExplorationPlanner/utils/RNG.h"
#include <boost/random/uniform_real_distribution.hpp>
/**
  * class PlanarRobotVelCmdSamplerUniform
  *
  */

class PlanarRobotVelCmdSamplerUniform : virtual public IPlanarRobotVelCmdSampler
{
public:

    // Constructors/Destructors
    //
    PlanarRobotVelCmdSamplerUniform(double default_duration_secs = 1.0)
        : default_ctrl_duration_secs_(default_duration_secs)
    {
    }

    virtual std::unique_ptr<IPlanarRobotVelCmdSampler> clone() const
    {
        return std::unique_ptr<IPlanarRobotVelCmdSampler>( new PlanarRobotVelCmdSamplerUniform(default_ctrl_duration_secs_));
    }

    virtual PlanarRobotVelCmd sample(const PlanarRobotVelCmd& old_cmd,
                                     const IPlanarKinematics& kinematics,
                                     RNG &rng);
    std::string Print() const;

private:
    typedef boost::random::uniform_real_distribution<double> UniDist;
    double default_ctrl_duration_secs_;
};

#endif // PLANARROBOTVELCMDSAMPLERUNIFORM_H
