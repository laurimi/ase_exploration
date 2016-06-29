#ifndef IPLANNINGALGORITHM_H
#define IPLANNINGALGORITHM_H
#include "ExplorationTaskState.h"
#include "ExplorationPlanner/kinematics/PlanarPose.h"
#include <vector>

/**
  * class PlanningAlgorithm
  *
  */
class IPlanningAlgorithm
{
public:  
    virtual ~IPlanningAlgorithm() {}

    /**
   * @return PlanarPose
   * @param  initial_state
   */
    virtual bool plan(const ExplorationTaskState& state) = 0;
    virtual void getPlan(std::vector<PlanarPose> &plan) const = 0;
};

#endif // IPLANNINGALGORITHM_H
