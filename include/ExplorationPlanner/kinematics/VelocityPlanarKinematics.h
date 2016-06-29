#ifndef VELOCITYPLANARKINEMATICS_H
#define VELOCITYPLANARKINEMATICS_H
#include "IPlanarKinematics.h"
#include "IConstraint.h"
#include <string>
#include <memory>
#include <boost/shared_ptr.hpp>


/**
  * class VelocityPlanarKinematics
  *
  */

class VelocityPlanarKinematics : virtual public IPlanarKinematics
{
public:

    VelocityPlanarKinematics( const IConstraint& vc,
                              const IConstraint& ac,
                              const IConstraint& fc);

    virtual std::unique_ptr<IPlanarKinematics> clone() const;

    /**
   * @return bool
   * @param  cmd
   */
    virtual bool checkConstraints(const PlanarRobotVelCmd& cmd,
                                  const PlanarPose& current_pose) const;

    /**
   * @return PlanarPose
   * @param  current_pose
   * @param  cmd
   */
    virtual PlanarPose getNextPose(const PlanarPose& current_pose, const PlanarRobotVelCmd& cmd) const;

    virtual void getTrajectory(std::vector<PlanarPose>& trajectory, const PlanarPose& start_pose, const std::vector<PlanarRobotVelCmd>& commands) const;


    const IConstraint& getLinearVelC() const
    {
        return *linear_vel_c_;
    }

    const IConstraint& getAngularVelC() const
    {
        return *angular_vel_c_;
    }

    const IConstraint& getFinalRotC() const
    {
        return *final_rot_c_;
    }

private:
    std::unique_ptr<IConstraint> linear_vel_c_;
    std::unique_ptr<IConstraint> angular_vel_c_;
    std::unique_ptr<IConstraint> final_rot_c_;
};

#endif // VELOCITYPLANARKINEMATICS_H
