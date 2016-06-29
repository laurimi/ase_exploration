#ifndef PLANARPOSE_H
#define PLANARPOSE_H
#include <string>
/**
  * class PlanarPose
  *
  */

class PlanarPose
{
public:

    // Constructors/Destructors
    //

    PlanarPose()
        : x_(0.0), y_(0.0), theta_(0.0)
    {
    }

    PlanarPose(double x, double y, double theta)
        : x_(x), y_(y), theta_(theta)
    {
    }

    // copy ctor
    PlanarPose(const PlanarPose& p)
        : x_(p.x_), y_(p.y_), theta_(p.theta_)
    {

    }

    std::string Print() const;

private:
    double x_;
    double y_;
    double theta_;

public:


    // Private attribute accessor methods
    //
    /**
   * Get the value of x_
   * @return the value of x_
   */
    double getX() const  {
        return x_;
    }

    /**
   * Get the value of y_
   * @return the value of y_
   */
    double getY() const  {
        return y_;
    }

    /**
   * Get the value of theta_
   * @return the value of theta_
   */
    double getTheta() const  {
        return theta_;
    }
};

#endif // PLANARPOSE_H
