#include "ExplorationPlanner/trajectory/TrajectoryGenerator.h"

bool TrajectoryGenerator::generate(std::vector<PlanarPose> &trajectory,
                                   std::vector<PlanarRobotVelCmd> &commands,
                                   IPlanarRobotVelCmdSampler &sampler,
                                   RNG &rng) const
{
    // first item in trajectory is curren pose
    std::size_t sz = commands.size();
    if (sz == 0 || sz != (trajectory.size() - 1) )
    {
        std::cout << "Unexpected input size for TrajectoryGenerator::generate\n";
        return false;
    }

    double total_distance = 0.0;

    for ( std::size_t is = 0; is < sz; ++is)
    {
        PlanarPose current_pose(trajectory[is]);
        PlanarRobotVelCmd current_cmd(commands[is]);

        if ( !updatePoseAndCmd(current_pose, current_cmd, sampler, rng)  )
        {
            // stay in place
            trajectory[is+1] = trajectory[is];
            commands[is] = PlanarRobotVelCmd(0.0, 0.0, 0.0, current_cmd.getDuration());
        }
        else
        {
            trajectory[is+1] = current_pose;
            commands[is] = current_cmd;
        }

        total_distance += std::abs( commands[is].getLinearVelocity() * commands[is].getDuration() );
    }

    return true;
}

// newpose and newcmd are only modified if return value is 'true'
bool TrajectoryGenerator::updatePoseAndCmd(PlanarPose& newpose,
                                           PlanarRobotVelCmd& newcmd,
                                           IPlanarRobotVelCmdSampler& sampler,
                                           RNG& rng) const
{
    unsigned int retries = max_tries_;
    do
    {
        PlanarRobotVelCmd candidate_cmd = sampler.sample(newcmd, *kinematics_, rng);
        std::vector<PlanarPose> subparts;
        getSubParts(subparts, newpose, candidate_cmd);

        if ( checker_->isTrajectoryValid(subparts) )
        { // trajectory is valid, we're done
            newpose = subparts.back();
            newcmd = candidate_cmd;
            return true;
        }
        else
        {
            --retries;
        }
    } while (retries > 0);

    return false;
}


void TrajectoryGenerator::getSubParts(std::vector<PlanarPose>& subparts,
                                      const PlanarPose& start_pose,
                                      const PlanarRobotVelCmd& command) const
{
    double d = std::abs( command.getLinearVelocity() * command.getDuration() );
    unsigned int num_parts = 2 * checker_->getNumOfCellsWithinDistance(d); // safety margin: multiply by two
    double dt = command.getDuration() / num_parts;
    std::vector<PlanarRobotVelCmd> split_cmd(num_parts, PlanarRobotVelCmd(command.getLinearVelocity(),
                                                                          command.getAngularVelocity(),
                                                                          command.getFinalRotation(),
                                                                          dt));
    kinematics_->getTrajectory(subparts, start_pose, split_cmd);
}
