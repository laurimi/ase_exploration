#ifndef IPLANARKINEMATICS_H
#define IPLANARKINEMATICS_H
#include "PlanarPose.h"
#include "PlanarRobotVelCmd.h"
#include "IConstraint.h"
#include <vector>
#include <string>
#include <memory>
/**
  * class IPlanarKinematics
  *
  */

class IPlanarKinematics
{
public:
    virtual ~IPlanarKinematics() {}

    virtual std::unique_ptr<IPlanarKinematics> clone() const = 0;

    /**
   * @return bool
   * @param  cmd
   */
    virtual bool checkConstraints(const PlanarRobotVelCmd& cmd,
                                  const PlanarPose& current_pose) const = 0;

    /**
   * @return PlanarPose
   * @param  current_pose
   * @param  cmd
   */
    virtual PlanarPose getNextPose(const PlanarPose& current_pose, const PlanarRobotVelCmd& cmd) const = 0;
    virtual void getTrajectory(std::vector<PlanarPose>& trajectory,
                               const PlanarPose& start_pose,
                               const std::vector<PlanarRobotVelCmd>& commands) const = 0;


    virtual const IConstraint& getLinearVelC() const = 0;
    virtual const IConstraint& getAngularVelC() const = 0;
    virtual const IConstraint& getFinalRotC() const = 0;

protected:
private:

};

#endif // IPLANARKINEMATICS_H
