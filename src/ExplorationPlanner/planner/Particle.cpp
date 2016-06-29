#include "ExplorationPlanner/planner/Particle.h"
#include <sstream>
// Constructors/Destructors
//  

//  
// Methods
//  
//void Particle::update(const PlanarPose &pose,
//                      const PlanarGridOccupancyMap& map,
//                      IPlanarRobotVelCmdSampler &k,
//                      const IPlanarKinematics& kinematics,
//                      RNG &rng)
//{
//    PlanarPose trajPose(pose);
//    for( std::vector<PlanarRobotVelCmd>::iterator it = cmds_.begin(), iend = cmds_.end(); it != iend; ++it)
//        *it = k.sample(trajPose, map, *it, kinematics, rng); // trajPose will update for each control
//}

std::string Particle::Print() const
{
    std::stringstream ss;
    ss << "Weight " << weight_ << "\n";
    const unsigned int sz = cmds_.size();
    ss << "Number of controls: " << sz << "\n";
    for(unsigned int i = 0; i < sz; ++i)
        ss << cmds_[i].Print() << "\n";
    const unsigned int st = trajectory_.size();
    ss << "Number of trajectory parts: " << st << "\n";
    for(unsigned int i = 0; i < st; ++i)
        ss << trajectory_[i].Print() << "\n";
    return ss.str();
}

