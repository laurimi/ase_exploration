#ifndef EXPLORATIONTASKSTATE_H
#define EXPLORATIONTASKSTATE_H
#include "ExplorationPlanner/kinematics/PlanarPose.h"
#include "ExplorationPlanner/grid/PlanarGridOccupancyMap.h"
#include "ExplorationPlanner/sensor/PlanarObservation.h"
/**
  * class ExplorationTaskState
  *
  */

class ExplorationTaskState
{
public:

    // Constructors/Destructors
    //
    ExplorationTaskState(PlanarPose robot_pose, PlanarGridOccupancyMap map);

    void updateMap(const PlanarObservation& obs)
    {
        map_.update(obs);
    }

private:

    // Static Private attributes
    //

    // Private attributes
    //

    PlanarPose robot_pose_;
    PlanarGridOccupancyMap map_;
public:


    // Private attribute accessor methods
    //
    /**
     * Set the value of robot_pose_
     * @param pose the new value of robot_pose_
     */
    void setPose(const PlanarPose& pose) {
        robot_pose_ = pose;
    }


    /**
   * Get the value of robot_pose_
   * @return the value of robot_pose_
   */
    PlanarPose getRobotPose() const  {
        return robot_pose_;
    }

    /**
   * Get reference to map_
   * @return reference to map_
   */
    const PlanarGridOccupancyMap& getMap() const {
        return map_;
    }

};

#endif // EXPLORATIONTASKSTATE_H
