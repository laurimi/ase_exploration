#ifndef PARTICLE_H
#define PARTICLE_H
#include "ExplorationPlanner/kinematics/PlanarRobotVelCmd.h"
#include "ExplorationPlanner/kinematics/IPlanarRobotVelCmdSampler.h"
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

/**
  * class Particle
  *
  */

class Particle
{
public:

    // Constructors/Destructors
    //
    Particle()
        : weight_(0.0),
          trajectory_(),
          cmds_()
    {

    }

    Particle(double w,
             const std::vector<PlanarPose>& trajectory,
             const std::vector<PlanarRobotVelCmd>& cmds)
        : weight_(w),
          trajectory_(trajectory),
          cmds_(cmds)
    {

    }

    std::vector<PlanarPose>& getTrajectory() {
        return trajectory_;
    }

    std::vector<PlanarRobotVelCmd>& getCommands() {
        return cmds_;
    }

    const std::vector<PlanarPose>& getTrajectory() const {
        return trajectory_;
    }

    const std::vector<PlanarRobotVelCmd>& getCommands() const {
        return cmds_;
    }

    /**
   * Set the value of weight_
   * @param w the new value of weight_
   */
    void setWeight(double w)  {
        weight_ = w;
    }

    /**
   * Get the value of weight_
   * @return the value of weight_
   */
    double getWeight() const {
        return weight_;
    }


    std::string Print() const;

private:
    // Private attributes
    //
    double weight_;
    std::vector<PlanarPose> trajectory_;
    std::vector<PlanarRobotVelCmd> cmds_;
};

#endif // PARTICLE_H
