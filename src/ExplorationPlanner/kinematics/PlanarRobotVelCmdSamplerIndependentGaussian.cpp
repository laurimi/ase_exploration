#include "ExplorationPlanner/kinematics/PlanarRobotVelCmdSamplerIndependentGaussian.h"
#include <sstream>


// Constructors/Destructors
//  
PlanarRobotVelCmdSamplerIndependentGaussian::PlanarRobotVelCmdSamplerIndependentGaussian(double linear_velocity_stdev,
                                                                                         double angular_velocity_stdev,
                                                                                         double final_rotation_stdev,
                                                                                         double default_duration_secs)
    : default_ctrl_duration_secs_(default_duration_secs),
      lv_std_(linear_velocity_stdev),
      av_std_(angular_velocity_stdev),
      fr_std_(final_rotation_stdev)
{
}

PlanarRobotVelCmd PlanarRobotVelCmdSamplerIndependentGaussian::sample(const PlanarRobotVelCmd& old_cmd,
                                 const IPlanarKinematics& kinematics,
                                 RNG &rng)
{
    NormalDist dv( old_cmd.getLinearVelocity(), lv_std_);
    NormalDist da( old_cmd.getAngularVelocity(), av_std_);
    NormalDist dr( old_cmd.getFinalRotation(), fr_std_);

    const double v = kinematics.getLinearVelC().forceToNearestLimit( rng(dv)  );
    const double a = kinematics.getAngularVelC().forceToNearestLimit( rng(da)  );
    const double r = kinematics.getFinalRotC().forceToNearestLimit( rng(dr)  );
    return PlanarRobotVelCmd(v, a, r, default_ctrl_duration_secs_ );
}

std::string PlanarRobotVelCmdSamplerIndependentGaussian::Print() const
{
    std::stringstream ss;
    ss << "Independent Gaussian kernel\n";
    ss << "Standard deviations:\n";
    ss << "  Linear velocity : " << lv_std_ << "\n";
    ss << "  Angular velocity: " << av_std_ << "\n";
    ss << "  Final rotation  : " << fr_std_ << "\n";
    return ss.str();
}

