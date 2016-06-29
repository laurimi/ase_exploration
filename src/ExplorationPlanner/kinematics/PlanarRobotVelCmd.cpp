#include "ExplorationPlanner/kinematics/PlanarRobotVelCmd.h"
#include <sstream>

std::string PlanarRobotVelCmd::Print() const
{
    std::stringstream ss;
    ss << "Planar robot velocity command\n";
    ss << "    Linear velocity : " << linear_vel_mps_ << " m/s\n";
    ss << "    Angular velocity: " << angular_vel_rps_ << " rad/s\n";
    ss << "    Final rotation  : " << final_rotation_rads << " rad/s\n";
    ss << "    Duration        : " << duration_secs_ << " s\n";
    return ss.str();
}


