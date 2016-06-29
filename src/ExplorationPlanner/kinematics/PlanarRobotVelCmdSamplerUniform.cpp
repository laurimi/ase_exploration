#include "ExplorationPlanner/kinematics/PlanarRobotVelCmdSamplerUniform.h"
#include <sstream>

PlanarRobotVelCmd PlanarRobotVelCmdSamplerUniform::sample(const PlanarRobotVelCmd& old_cmd,
                                                                  const IPlanarKinematics& kinematics,
                                                                  RNG &rng)
{
    UniDist dv( kinematics.getLinearVelC().getMinValue(), kinematics.getLinearVelC().getMaxValue()  );
    UniDist da( kinematics.getAngularVelC().getMinValue(), kinematics.getAngularVelC().getMaxValue()  );
    UniDist dr( kinematics.getFinalRotC().getMinValue(), kinematics.getFinalRotC().getMaxValue()  );

    return PlanarRobotVelCmd( rng(dv),
                              rng(da),
                              rng(dr),
                              default_ctrl_duration_secs_  );

}

std::string PlanarRobotVelCmdSamplerUniform::Print() const
{
    std::stringstream ss;
    ss << "Uniform kernel on space of allowed controls";
    return ss.str();
}
