#include "ExplorationPlanner/kinematics/VelocityPlanarKinematics.h"
#include "ExplorationPlanner/grid/PlanarGridIndex.h"
#include <boost/math/constants/constants.hpp>

VelocityPlanarKinematics::VelocityPlanarKinematics(const IConstraint& vc, const IConstraint &ac, const IConstraint &fc)
    : linear_vel_c_(vc.clone()),
      angular_vel_c_(ac.clone()),
      final_rot_c_(fc.clone())
{
}

std::unique_ptr<IPlanarKinematics> VelocityPlanarKinematics::clone() const
{
    return std::unique_ptr<IPlanarKinematics>( new VelocityPlanarKinematics( *linear_vel_c_,
                                                                             *angular_vel_c_,
                                                                             *final_rot_c_) );
}

bool VelocityPlanarKinematics::checkConstraints(const PlanarRobotVelCmd& cmd,
                                                const PlanarPose& current_pose) const
{
    return ( linear_vel_c_->check( cmd.getLinearVelocity() ) &&
             angular_vel_c_->check( cmd.getAngularVelocity()) &&
             final_rot_c_->check( cmd.getFinalRotation() ));
}


PlanarPose VelocityPlanarKinematics::getNextPose(const PlanarPose& current_pose, const PlanarRobotVelCmd& cmd) const
{
    const double angvel = cmd.getAngularVelocity();
    // get ratio of linear velocity and angular velocity
    double vw = cmd.getLinearVelocity();
    if ( angvel != 0.0 )
        vw /=  angvel;

    const double dt = cmd.getDuration();
    const double old_theta = current_pose.getTheta();
    const double next_x = current_pose.getX() - vw * sin(old_theta) + vw * sin(old_theta + angvel*dt);
    const double next_y = current_pose.getY() + vw * cos(old_theta) - vw * cos(old_theta + angvel*dt);
    double next_theta = current_pose.getTheta() + angvel * dt + cmd.getFinalRotation() * dt;

    // Unwrap the theta angle if necessary
    while ( next_theta > boost::math::constants::pi<double>() )
        next_theta -= 2*boost::math::constants::pi<double>();
    while ( next_theta < -boost::math::constants::pi<double>() )
        next_theta += 2*boost::math::constants::pi<double>();

    return PlanarPose(next_x, next_y, next_theta);
}

void VelocityPlanarKinematics::getTrajectory(std::vector<PlanarPose>& trajectory, const PlanarPose& start_pose, const std::vector<PlanarRobotVelCmd>& commands) const
{
    std::size_t sc = commands.size();
    trajectory.resize(sc+1, start_pose);
    trajectory[0] = start_pose;
    for (std::size_t is = 0; is < sc; ++is)
        trajectory[is+1] = getNextPose(trajectory[is], commands[is]);
}
