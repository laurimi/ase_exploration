#ifndef PLANARROBOTVELCMD_H
#define PLANARROBOTVELCMD_H
#include <string>

/**
  * class PlanarRobotVelCmd
  *
  */

class PlanarRobotVelCmd
{
public:

    // Constructors/Destructors
    //
    PlanarRobotVelCmd()
        : linear_vel_mps_(0.0), angular_vel_rps_(0.0),
          final_rotation_rads(0.0), duration_secs_(0.0)
    {
    }

    PlanarRobotVelCmd(double vel_lin, double vel_ang, double final_rot, double duration)
        : linear_vel_mps_(vel_lin), angular_vel_rps_(vel_ang),
          final_rotation_rads(final_rot), duration_secs_(duration)
    {
    }


    std::string Print() const;

    // Private attribute accessor methods
    //
    /**
   * Get the value of linear_vel_mps_
   * @return the value of linear_vel_mps_
   */
    double getLinearVelocity () const  {
        return linear_vel_mps_;
    }

    /**
   * Get the value of angular_vel_rps_
   * @return the value of angular_vel_rps_
   */
    double getAngularVelocity ()  const {
        return angular_vel_rps_;
    }

    /**
   * Get the value of final_rotation_rads
   * @return the value of final_rotation_rads
   */
    double getFinalRotation() const  {
        return final_rotation_rads;
    }


    /**
   * Get the value of duration_secs_
   * @return the value of duration_secs_
   */
    double getDuration() const  {
        return duration_secs_;
    }

private:
    // Private attributes
    //
    double linear_vel_mps_;
    double angular_vel_rps_;
    double final_rotation_rads;
    double duration_secs_;
};

#endif // PLANARROBOTVELCMD_H
