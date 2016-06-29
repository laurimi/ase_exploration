#ifndef SDMPLANNER_H
#define SDMPLANNER_H
#include "IPlanningAlgorithm.h"

/**
  * class SDMPlanner
  *
  */

/******************************* Abstract Class ****************************
SDMPlanner does not have any pure virtual methods, but its author
  defined it as an abstract class, so you should not use it directly.
  Inherit from it instead and create only objects from the derived classes
*****************************************************************************/

class SDMPlanner : virtual public IPlanningAlgorithm
{
public:

    // Constructors/Destructors
    //
    SDMPlanner(unsigned int horizon, double discount)
        : IPlanningAlgorithm(),
          horizon_(horizon),
          discount_(discount)
    {

    }

private:

    // Private attributes
    //

    unsigned int horizon_;
    double discount_;

public:


    // Private attribute accessor methods
    //


    /**
   * Get the value of horizon_
   * @return the value of horizon_
   */
    unsigned int getHorizon() const  {
        return horizon_;
    }

    /**
   * Get the value of discount_
   * @return the value of discount_
   */
    double getDiscount() const  {
        return discount_;
    }

};

#endif // SDMPLANNER_H
