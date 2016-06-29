#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H
#include "TrajectoryChecker.h"
#include "ExplorationPlanner/kinematics/PlanarPose.h"
#include "ExplorationPlanner/kinematics/PlanarRobotVelCmd.h"
#include "ExplorationPlanner/kinematics/IPlanarRobotVelCmdSampler.h"
#include "ExplorationPlanner/kinematics/IPlanarKinematics.h"
#include "ExplorationPlanner/utils/RNG.h"
#include <vector>

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(const IPlanarKinematics& kinematics,
                        const boost::shared_ptr<TrajectoryChecker>& checker,
                        double minimum_trajectory_length,
                        unsigned int max_tries)
        : kinematics_(kinematics.clone()),
          checker_(checker),
          minimum_trajectory_length_(minimum_trajectory_length),
          max_tries_(max_tries)
    {}


    //copy ctor
    TrajectoryGenerator(const TrajectoryGenerator& g)
        : kinematics_(g.kinematics_->clone()),
          checker_(g.checker_),
          minimum_trajectory_length_(g.minimum_trajectory_length_),
          max_tries_(g.max_tries_)
    {}


    // Fill trajectory and commands
    bool generate(std::vector<PlanarPose>& trajectory,
                  std::vector<PlanarRobotVelCmd>& commands,
                  IPlanarRobotVelCmdSampler& sampler,
                  RNG& rng) const;

private:
    double minimum_trajectory_length_;
    unsigned int max_tries_;

    std::unique_ptr<IPlanarKinematics> kinematics_;
    boost::shared_ptr<TrajectoryChecker> checker_;

    void getSubParts(std::vector<PlanarPose> &subparts,
                     const PlanarPose &start_pose,
                     const PlanarRobotVelCmd& command) const;

    bool updatePoseAndCmd(PlanarPose& newpose,
                          PlanarRobotVelCmd& newcmd,
                          IPlanarRobotVelCmdSampler& sampler,
                          RNG& rng) const;

};

#endif /* TRAJECTORYGENERATOR_H */
