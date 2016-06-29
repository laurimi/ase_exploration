#ifndef SMCPLANNER_H
#define SMCPLANNER_H
#include "SMCPlannerParameters.h"
#include "ParticleSet.h"
#include "SDMPlanner.h"
#include "ExplorationPlanner/trajectory/TrajectoryEvaluator.h"
#include "ExplorationPlanner/trajectory/TrajectoryGenerator.h"
#include "ExplorationPlanner/utils/RNG.h"
#include <vector>
#include <memory>

/**
  * class SMCPlanner
  *
  */

class SMCPlanner : virtual public SDMPlanner
{
public:
    SMCPlanner(unsigned int horizon,
               double discount,
               const boost::shared_ptr<TrajectoryGenerator>& generator,
               const boost::shared_ptr<TrajectoryEvaluator>& evaluator,
               const SMCPlannerParameters& params)
        : IPlanningAlgorithm(),
          SDMPlanner(horizon, discount),
          generator_(generator),
          evaluator_(evaluator),
          parameters_(params.clone()),
          particleset_(),
          rng_(),
          iteration_(0)
    {

    }

    bool iterate(const ExplorationTaskState& state);



    /**
   * @return PlanarPose
   * @param  initial_state
   */
    bool plan(const ExplorationTaskState& state);
    void getPlan(std::vector<PlanarPose> &plan) const;

    void setParameters(const SMCPlannerParameters& p)
    {
        parameters_ = p.clone();
    }


    // At least use in debugging phase: get all
    const ParticleSet& getParticleSet() const
    {
        return particleset_;
    }

    std::vector<TrajectoryValueHandler> getLatestRewardHandlers() const {
        return latest_rewards_;
    }

private:

    // Private attributes
    //
    boost::shared_ptr<TrajectoryGenerator> generator_;
    boost::shared_ptr<TrajectoryEvaluator> evaluator_;
    std::unique_ptr<SMCPlannerParameters> parameters_;

    ParticleSet particleset_;
    RNG rng_;

    unsigned int iteration_;

    void initializeParticles();
    bool updateParticles();
    void evaluateParticles(const ExplorationTaskState& state);


    std::vector<TrajectoryValueHandler> latest_rewards_;
};

#endif // SMCPLANNER_H
