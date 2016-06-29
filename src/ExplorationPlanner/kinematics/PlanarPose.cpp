#include "ExplorationPlanner/kinematics/PlanarPose.h"
#include <sstream>
// Constructors/Destructors
//  

//  
// Methods
//  
std::string PlanarPose::Print() const
{
    std::stringstream ss;
    ss << "Planar pose\n";
    ss << "  x    : " << x_ << "\n";
    ss << "  y    : " << y_ << "\n";
    ss << "  theta: " << theta_;
    return ss.str();
}


