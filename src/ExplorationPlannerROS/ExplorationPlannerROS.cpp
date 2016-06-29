#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>
#include <algorithm>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "ExplorationPlannerROS/ExplorationPlannerROS.h"
#include "ExplorationPlanner/trajectory/TrajectoryChecker.h"
#include "ExplorationPlanner/trajectory/TrajectoryGenerator.h"
#include "ExplorationPlanner/kinematics/RangeConstraint.h"
#include "ExplorationPlanner/kinematics/VelocityPlanarKinematics.h"
#include "ExplorationPlanner/sensor/LaserScanner2D.h"
#include "ExplorationPlanner/planner/LinearCoolingSchedule.h"
#include "ExplorationPlanner/kinematics/PlanarRobotVelCmdSamplerUniform.h"
#include "ExplorationPlanner/kinematics/PlanarRobotVelCmdSamplerIndependentGaussian.h"
#include "ExplorationPlanner/planner/SMCPlanner.h"

// Frontier exploration services
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>

namespace explorationplanner_ros {

ExplorationPlannerROS::ExplorationPlannerROS(const std::string name)
    : nh_(""),
      private_nh_("~"),
      tf_listener_(ros::Duration(10.0)),
      move_client_("move_base",true),
      explore_server_(nh_, name, boost::bind(&ExplorationPlannerROS::executeCb, this, _1), false),
      moving_(false),
      possibly_stuck_(false),
      goal_is_frontier_(false),
      global_map_(PlanarPose(0.0, 0.0, 0.0), 0.05)
{
    // Read parameters and prepare the object for use accordingly
    readParameters();

    // at this point, we can update trajectory evaluator (it uses only laser parameters)
    updateTrajectoryEvaluator();

    // Subscribe to costmap from move_base and a global map from e.g. SLAM
    costmap_sub_ = nh_.subscribe("costmap", 1, &ExplorationPlannerROS::costmapCb, this);
    costmap_update_sub_ = nh_.subscribe("costmap_updates", 1, &ExplorationPlannerROS::costmapUpdateCb, this);

    global_map_sub_ = nh_.subscribe("map", 1, &ExplorationPlannerROS::globalmapCb, this);

    // dynamic reconfiguration setup
    confCb_ = boost::bind(&ExplorationPlannerROS::confCb, this, _1, _2);
    conf_server_.setCallback(confCb_);

    // Action server start
    explore_server_.registerPreemptCallback(boost::bind(&ExplorationPlannerROS::preemptCb, this));
    explore_server_.start();

    // paths inspected by the planner
    path_pub_ = nh_.advertise<nav_msgs::Path>("planner_paths", 10, true);
}

void ExplorationPlannerROS::readParameters()
{
    private_nh_.param("wait_between_planner_iterations", params_.wait_between_planner_iterations, false);

    private_nh_.param("lin_v_min", params_.lin_v_min, 0.3);
    private_nh_.param("lin_v_max", params_.lin_v_max, 1.0);
    private_nh_.param("ang_v_min", params_.ang_v_min, -1.0);
    private_nh_.param("ang_v_max", params_.ang_v_max, 1.0);
    private_nh_.param("f_rot_min", params_.f_rot_min, -0.02);
    private_nh_.param("f_rot_max", params_.f_rot_max, 0.02);

    private_nh_.param("min_traj_length", params_.min_traj_length, 0.5);
    private_nh_.param("min_reward", params_.min_reward, 50.0);
    private_nh_.param("max_sampling_tries", params_.max_sampling_tries, 30);

    private_nh_.param("laser_min_angle_deg", params_.laser_min_angle_deg, -90.0);
    private_nh_.param("laser_max_angle_deg", params_.laser_max_angle_deg, 90.0);
    private_nh_.param("laser_angle_step_deg", params_.laser_angle_step_deg, 5.0);
    private_nh_.param("laser_max_dist_m", params_.laser_max_dist_m, 4.0);
    private_nh_.param("p_false_pos", params_.p_false_pos, 0.05);
    private_nh_.param("p_false_neg", params_.p_false_neg, 0.05);

    private_nh_.param("horizon", params_.horizon, 3);
    private_nh_.param("discount", params_.discount, 1.0);

    private_nh_.param("schedule_a", params_.schedule_a, 3);
    private_nh_.param("schedule_b", params_.schedule_b, 3);
    private_nh_.param("num_kernels", params_.num_kernels, 5);

    private_nh_.param("num_particles", params_.num_particles, 10);
    ROS_ASSERT(params_.num_particles>0);
    private_nh_.param("resample_thresh", params_.resample_thresh, 0.33);

    private_nh_.param("std_vel", params_.std_vel, 0.2);
    private_nh_.param("std_ang", params_.std_ang, 0.1);
    private_nh_.param("std_fr", params_.std_fr, 0.02);
    private_nh_.param("default_ctrl_duration", params_.default_ctrl_duration, 1.0);

    private_nh_.param("allow_unknown_targets", params_.allow_unknown_targets, true);

    private_nh_.param("goal_execution_time", params_.goal_execution_time, 10.0);
    private_nh_.param("frontier_distance_threshold", params_.frontier_distance_threshold, 2.0);

    private_nh_.param<std::string>("map_frame_id", params_.map_frame_id_, "map");
    private_nh_.param<std::string>("base_frame_id", params_.base_frame_id_, "base_link");
}

void ExplorationPlannerROS::updateTrajectoryGenerator()
{
    boost::mutex::scoped_lock lock(costmap_lock_);

    // Update the constraints
    RangeConstraint linvel_c(params_.lin_v_min, params_.lin_v_max);
    RangeConstraint angvel_c(params_.ang_v_min, params_.ang_v_max);
    RangeConstraint rot_c(params_.f_rot_min, params_.f_rot_max);
    VelocityPlanarKinematics kin(linvel_c, angvel_c, rot_c);

    // rebuild object with new trajectory checker
    boost::shared_ptr<TrajectoryChecker>
            chk( new TrajectoryChecker(trajectorycheck_map_, params_.allow_unknown_targets));

    // construct new trajectory generator object
    trajgenerator_ = boost::shared_ptr<TrajectoryGenerator>(
                new TrajectoryGenerator(kin,
                                        chk,
                                        params_.min_traj_length,
                                        params_.max_sampling_tries)
                );
}

void ExplorationPlannerROS::updateTrajectoryEvaluator()
{
    LaserScanner2D laser( params_.laser_min_angle_deg*(boost::math::constants::pi<double>()/180),
                          params_.laser_max_angle_deg*(boost::math::constants::pi<double>()/180),
                          params_.laser_angle_step_deg*(boost::math::constants::pi<double>()/180),
                          params_.laser_max_dist_m,
                          params_.p_false_pos,
                          params_.p_false_neg);
    trajevaluator_ = boost::shared_ptr<TrajectoryEvaluator>( new TrajectoryEvaluator(laser) );
}


std::unique_ptr<SMCPlanner> ExplorationPlannerROS::buildPlanner() const
{
    ROS_DEBUG("Building planner object...");
    LinearCoolingSchedule schedule(params_.schedule_a, params_.schedule_b);
    SMCPlannerParameters param(schedule, params_.num_particles, params_.resample_thresh);

    PlanarRobotVelCmdSamplerUniform firstKC(params_.default_ctrl_duration);
    param.addKernel( firstKC );
    for (unsigned int i = 0; i < params_.num_kernels; ++i)
    {
        const double ls = params_.std_vel  / (i+1);
        const double as = params_.std_ang / (i+1);
        const double fs = params_.std_fr / (i+1);
        PlanarRobotVelCmdSamplerIndependentGaussian kg(ls, as, fs, params_.default_ctrl_duration);
        param.addKernel( kg );
    }
    std::unique_ptr<SMCPlanner> smc(new SMCPlanner(params_.horizon, params_.discount, trajgenerator_, trajevaluator_, param));
    ROS_DEBUG("Planner object built");
    return smc;
}


void ExplorationPlannerROS::confCb(ase_exploration::PlannerConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request, level: %d", level);
    if (level == 0)
    {
        ROS_INFO("Planner parameters reconfigured");
        params_.horizon = config.horizon;
        params_.discount = config.discount;
        params_.schedule_a = config.schedule_a;
        params_.schedule_b = config.schedule_b;
        params_.num_kernels = config.num_kernels;
        params_.std_vel = config.std_vel;
        params_.std_ang = config.std_ang;
        params_.std_fr = config.std_fr;
        params_.default_ctrl_duration = config.default_ctrl_duration;
        params_.num_particles = config.num_particles;
        params_.resample_thresh = config.resample_thresh;
    }
    else if (level == 1)
    {
        ROS_DEBUG("Sensor parameters reconfigured");
        params_.laser_min_angle_deg = config.laser_min_angle_deg;
        params_.laser_max_angle_deg = config.laser_max_angle_deg;
        params_.laser_angle_step_deg = config.laser_angle_step_deg;
        params_.laser_max_dist_m = config.laser_max_dist_m;
        params_.p_false_pos = config.laser_p_false_pos;
        params_.p_false_neg = config.laser_p_false_neg;
        updateTrajectoryEvaluator();
    }
    else if ( level == 2)
    {
        ROS_DEBUG("Trajectory generation parameters reconfigured");
        params_.allow_unknown_targets = config.allow_unknown_targets;
    }
    else
        ROS_INFO("Undefined reconfiguration request");
}

void ExplorationPlannerROS::executeCb(const ase_exploration::ExploreGoalConstPtr &goal)
{
    ROS_INFO("Received new exploration task");
    moving_ = false;
    possibly_stuck_ = false;
    goal_is_frontier_ = false;

    // wait for move_base to become available
    if(!move_client_.waitForServer() )
    {
        explore_server_.setAborted();
        return;
    }

    //loop until the task is finished or pre-empted
    while(ros::ok() && explore_server_.isActive())
    {
        // get robot's current pose
        tf::StampedTransform tf_map_to_robot_base;
        try
        {
            tf_listener_.lookupTransform(params_.map_frame_id_, params_.base_frame_id_,
                                     ros::Time(0), tf_map_to_robot_base);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("TF Error: %s",ex.what());
            ROS_ERROR("Exploration failed");
            explore_server_.setAborted();
            return;
        }
        PlanarPose robot_planar_pose(tf_map_to_robot_base.getOrigin().getX(),
                                     tf_map_to_robot_base.getOrigin().getY(),
                                     tf::getYaw(tf_map_to_robot_base.getRotation()) );

        // Update the trajectory generator for this round
        updateTrajectoryGenerator();
        std::unique_ptr<SMCPlanner> planner = buildPlanner();

        // get map information to use in planning
        global_map_lock_.lock();
        ExplorationTaskState s(robot_planar_pose, global_map_ );
        global_map_lock_.unlock();

        // plan for requested number of steps
        std::vector<double> avgRewards, minRewards, maxRewards, pathDist;
        unsigned int planner_iteration = 0;
        while ( (planner_iteration < params_.num_kernels) && explore_server_.isActive() )
        {
            ros::Time t0 = ros::Time::now();
            if ( !planner->iterate(s) )
                ROS_WARN("Planner iteration failed");
            ros::Time t1 = ros::Time::now();
            ROS_INFO("Planner iteration %d complete in %f s", planner_iteration, (t1-t0).toSec());

            // publish reward and path information
            ParticleSet ps = planner->getParticleSet();
            std::vector<TrajectoryValueHandler> rh = planner->getLatestRewardHandlers();
            std::stringstream ss, sr, smin, smax;
            for (unsigned int i = 0; i < ps.getNumberOfParticles(); ++i)
            {
                std::vector<PlanarPose> path = ps[i].getTrajectory();
                publishPath(path);

                ss << ps[i].getWeight() << ", ";
                smin << rh[i].getMinSumOfDiscRew() << ", ";
                smax << rh[i].getMaxSumOfDiscRew() << ", ";
                sr << rh[i].getAvgSumOfDiscRew() << ", ";

                // at last iteration, get summary information to decide what to do next
                if ( i == params_.num_kernels )
                {
                    avgRewards.push_back( rh[i].getAvgSumOfDiscRew() );
                    minRewards.push_back( rh[i].getMinSumOfDiscRew() );
                    maxRewards.push_back( rh[i].getMaxSumOfDiscRew() );


                    std::vector<PlanarPose> path = ps[i].getTrajectory();
                    PlanarPose target = path.end()[-2];
                    if ( path.size() == 2) // start pose + one other pose
                        target = path.back();

                    double dx = robot_planar_pose.getX() - target.getX();
                    double dy = robot_planar_pose.getY() - target.getY();
                    pathDist.push_back( std::sqrt( dx*dx + dy*dy  )  );
                }
            }

            // Print particle information
            ROS_DEBUG("Particle weights:           %s", ss.str().c_str());
            ROS_DEBUG("Minimum discounted rewards: %s", smin.str().c_str());
            ROS_DEBUG("Maximum discounted rewards: %s", smax.str().c_str());
            ROS_DEBUG("Average discounted rewards: %s", sr.str().c_str());

            if ( params_.wait_between_planner_iterations && (planner_iteration != params_.num_kernels) )
            {
                ROS_INFO("Press button to continue");
                std::cin.ignore();
            }
            ++planner_iteration;
        }

        // get the best plan from the planner
        std::vector<PlanarPose> plan;
        planner->getPlan(plan);

        // heuristics for checking if we might be stuck
        if ( std::all_of( avgRewards.begin(), avgRewards.end(), [this](double r){return r < params_.min_reward;} ) ||
             std::all_of( pathDist.begin(), pathDist.end(), [this](double d){return d < this->params_.min_traj_length; }   )
             )
        {
            ROS_WARN("All rewards less than minimum of %f required or target closer than given threshold %f meters, planner may be stuck!", params_.min_reward, params_.min_traj_length);
            possibly_stuck_ = true;
        }


        // Construct the goal pose message to be filled and sent to move_base
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = params_.map_frame_id_;

        if (!possibly_stuck_)
        {
            // Publish whole path for best particle
            // Get the goal pose -- move to second to last pose on the path (if it exists)
            // end() is past the last element, -1 for last element, -2 for second-last
            PlanarPose target = plan.end()[-2];
            if ( plan.size() == 2) // start pose + one other pose (horizon == 1)
                target = plan.back();

            goal_pose.pose.position.x = target.getX();
            goal_pose.pose.position.y = target.getY();
            tf::Quaternion q;
            q.setRPY( 0, 0, target.getTheta() );
            tf::quaternionTFToMsg(q, goal_pose.pose.orientation);

            // this goal is not a frontier
            goal_is_frontier_ = false;
        }
        else
        {
            // stuck - try to get a new target from frontier exploration
            ROS_INFO("Attempting recovery from stuck position via frontier exploration");

            // create the costmap service clients
            ros::ServiceClient updateBoundaryPolygon =
                    nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>
                    ("frontier_exploration/explore_costmap/explore_boundary/update_boundary_polygon");
            ros::ServiceClient getNextFrontier =
                    nh_.serviceClient<frontier_exploration::GetNextFrontier>
                    ("frontier_exploration/explore_costmap/explore_boundary/get_next_frontier");

            if (!updateBoundaryPolygon.waitForExistence() ||
                    !getNextFrontier.waitForExistence() )
            {
                ROS_ERROR("Services for frontier exploration not available; exploration task canceled");
                explore_server_.setAborted();
                return;
            }

            // set unbounded for frontier_exploration
            frontier_exploration::UpdateBoundaryPolygon bndsrv;
            bndsrv.request.explore_boundary.header.frame_id = params_.map_frame_id_;
            if(updateBoundaryPolygon.call(bndsrv))
            {
                ROS_DEBUG("Region boundary set for frontier_exploration");
            }
            else
            {
                ROS_ERROR("Failed to set region boundary for frontier_exploration");
                explore_server_.setAborted();
                return;
            }

            // Service call to frontier_exploration
            frontier_exploration::GetNextFrontier srv;
            // give current pose as start for frontier search
            srv.request.start_pose.header.frame_id = params_.map_frame_id_;
            srv.request.start_pose.pose.position.x = robot_planar_pose.getX();
            srv.request.start_pose.pose.position.y = robot_planar_pose.getY();
            tf::quaternionTFToMsg(tf_map_to_robot_base.getRotation(), srv.request.start_pose.pose.orientation);

            if ( getNextFrontier.call(srv) )
            {
                ROS_INFO("Got next frontier from frontier_exploration");
                goal_pose = srv.response.next_frontier;
            }
            else
            {
                ROS_ERROR("Call to service %s failed", getNextFrontier.getService().c_str() );
                explore_server_.setAborted();
                break;
            }
            // this goal is a frontier
            goal_is_frontier_ = true;
            possibly_stuck_ = false;
        }

        // pass goal to move_base
        feedback_.current_target = goal_pose;
        explore_server_.publishFeedback(feedback_);

        move_client_goal_.target_pose = goal_pose;
        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        goal_time_ = ros::Time::now();
        if (explore_server_.isActive())
        {
            move_client_.sendGoal(move_client_goal_,
                                  boost::bind(&ExplorationPlannerROS::doneMovingCb, this, _1, _2),
                                  0,
                                  boost::bind(&ExplorationPlannerROS::feedbackMovingCb, this, _1));
            moving_ = true;
        }
        lock.unlock();

        //wait for movement to finish before continuing
        while(ros::ok() && explore_server_.isActive() && moving_)
        {
            ros::WallDuration(0.1).sleep();
        }
    }

    ROS_ASSERT(!explore_server_.isActive());
}

void ExplorationPlannerROS::publishPath(const std::vector<PlanarPose>& p)
{
    nav_msgs::Path pm;
    pm.header.frame_id = params_.map_frame_id_;
    unsigned int id = 0;
    for (std::vector<PlanarPose>::const_iterator it = p.begin(), iend = p.end(); it != iend; ++it)
    {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = pm.header.frame_id;
        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = it->getX();
        ps.pose.position.y = it->getY();
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);
    }

    path_pub_.publish(pm);
}

void ExplorationPlannerROS::preemptCb()
{
    boost::unique_lock<boost::mutex> lock(move_client_lock_);
    move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
    ROS_WARN("Current exploration task cancelled");

    if(explore_server_.isActive()){
        explore_server_.setPreempted();
    }
}

void ExplorationPlannerROS::doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){

    if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
        ROS_ERROR("Failed to move");
        explore_server_.setAborted();
    }
    else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        moving_ = false;
    }
}

void ExplorationPlannerROS::feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_DEBUG("move_base feedback received");
    bool cancel_goal = false;

    // if this is "regular goal" or frontier goal, check if we already executed it for given execution time
    double time_spent_on_goal = (ros::Time::now() - goal_time_).toSec();
    if ( !goal_is_frontier_ && time_spent_on_goal > params_.goal_execution_time)
    {
        ROS_INFO("Current exploration task ran for execution time of %f seconds, cancel it and replan", params_.goal_execution_time);
        cancel_goal = true;
    }

    // if this goal is a frontier, check if we are "close" to it and can move to regular planning again
    double dx = move_client_goal_.target_pose.pose.position.x - feedback->base_position.pose.position.x;
    double dy = move_client_goal_.target_pose.pose.position.y - feedback->base_position.pose.position.y;
    double distance_to_target = std::sqrt( dx*dx + dy*dy );
    if ( goal_is_frontier_ && distance_to_target < params_.frontier_distance_threshold )
    {
        ROS_INFO("Current frontier closer than %f meters, cancel it and replan", params_.frontier_distance_threshold);
        cancel_goal = true;
    }

    if ( cancel_goal )
    {
        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        moving_ = false;
    }

}

void ExplorationPlannerROS::costmapCb(const nav_msgs::OccupancyGrid& map)
{
    boost::mutex::scoped_lock lock(costmap_lock_);

    if (params_.map_frame_id_ != map.header.frame_id)
    {
        ROS_ERROR("Incoming costmap on topic %s has frame %s, expected %s. Costmap not updated.",
                  costmap_sub_.getTopic().c_str(),
                  map.header.frame_id.c_str(),
                  params_.map_frame_id_.c_str());
        return;
    }

    // sparsify the costmap.
    // update instead of rewrite might be more efficient, but rewrite fast enough for now...
    PlanarPose map_origin(map.info.origin.position.x,
                          map.info.origin.position.y,
                          tf::getYaw(map.info.origin.orientation));
    trajectorycheck_map_ = boost::shared_ptr<PlanarGridBinaryMap>(
                new PlanarGridBinaryMap(map_origin, map.info.resolution));

    for (unsigned int h = 0; h < map.info.height; ++h)
    {
        const unsigned int h_offset = h * map.info.width;
        for (unsigned int w = 0; w < map.info.width; ++w)
        {
            // in ROS costmap_2d, costmap_2d::INSCRIBED obstacle corresponds
            // to occupancy 99. We set our threshold there.
            if ( map.data[h_offset + w] != -1 &&
                 map.data[h_offset + w] >= 99 )
            {
                trajectorycheck_map_->setGridValue( PlanarGridIndex(w,h), true);
            }
        }
    }
}

void ExplorationPlannerROS::costmapUpdateCb(const map_msgs::OccupancyGridUpdate &map_update)
{
    boost::mutex::scoped_lock lock(costmap_lock_);

    if (params_.map_frame_id_ != map_update.header.frame_id)
    {
        ROS_ERROR("Incoming costmap on topic %s has frame %s, expected %s. Costmap not updated.",
                  costmap_sub_.getTopic().c_str(),
                  map_update.header.frame_id.c_str(),
                  params_.map_frame_id_.c_str());
        return;
    }

    for (unsigned int h = 0; h < map_update.height; ++h)
    {
        const unsigned int h_offset = h * map_update.width;
        for (unsigned int w = 0; w < map_update.width; ++w)
        {
            // in ROS costmap_2d, costmap_2d::INSCRIBED obstacle corresponds
            // to occupancy 99. We set our threshold there.
            if ( map_update.data[h_offset + w] != -1 &&
                 map_update.data[h_offset + w] >= 99 )
            {
                trajectorycheck_map_->setGridValue( PlanarGridIndex( map_update.x + w, map_update.y + h), true);
            }
        }
    }
}


void ExplorationPlannerROS::globalmapCb(const nav_msgs::OccupancyGrid &map)
{
    boost::mutex::scoped_lock lock(global_map_lock_);

    if (params_.map_frame_id_ != map.header.frame_id)
    {
        ROS_ERROR("Incoming global map on topic %s has frame id: %s, expected frame id: %s. Global map not updated.",
                  global_map_sub_.getTopic().c_str(),
                  map.header.frame_id.c_str(),
                  params_.map_frame_id_.c_str());
        return;
    }

    // sparsify the incoming global map
    PlanarPose map_origin(map.info.origin.position.x,
                          map.info.origin.position.y,
                          tf::getYaw(map.info.origin.orientation));
    global_map_ = PlanarGridOccupancyMap(map_origin, map.info.resolution);
    for (unsigned int h = 0; h < map.info.height; ++h)
    {
        const unsigned int h_offset = h*map.info.width;
        for (unsigned int w = 0; w < map.info.width; ++w)
        {
            if ( map.data[h_offset+w] != -1) // -1 implicitly unknown
                global_map_.setGridValue( PlanarGridIndex(w,h),  map.data[h_offset+w]);
        }
    }
}

} // namespace explorationplanner_ros
