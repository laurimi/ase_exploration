#ifndef IPLANARROBOTVELCMDSAMPLER_H
#define IPLANARROBOTVELCMDSAMPLER_H
#include "PlanarRobotVelCmd.h"
#include "IPlanarKinematics.h"
#include "ExplorationPlanner/planner/ExplorationTaskState.h"
#include "ExplorationPlanner/utils/RNG.h"
#include <string>
#include <memory>

/**
  * class IPlanarRobotVelCmdSampler
  *
  */
class IPlanarRobotVelCmdSampler
{
public:
    virtual ~IPlanarRobotVelCmdSampler() {}

    /**
   * @return PlanarRobotVelCmd
   * @param  cmd
   */

    virtual PlanarRobotVelCmd sample(const PlanarRobotVelCmd& old_cmd,
                                     const IPlanarKinematics& kinematics,
                                     RNG &rng) = 0;
    virtual std::unique_ptr<IPlanarRobotVelCmdSampler> clone() const = 0;
    virtual std::string Print() const = 0;
};

#endif // IPLANARROBOTVELCMDSAMPLER_H
