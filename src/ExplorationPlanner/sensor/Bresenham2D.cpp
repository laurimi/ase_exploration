#include "ExplorationPlanner/sensor/Bresenham2D.h"
#include "ExplorationPlanner/grid/PlanarGridIndex.h"
#include <cmath>
#include <utility>

// Constructors/Destructors
//
Bresenham2D::Bresenham2D() :
    initialized_(false), x1(0), y1(0), x2(0), y2(0), delta_x_(0),
    delta_y_(0), error_(0), ix(0), iy(0), dx_geq_dy_(false)
{
}

//  
// Methods
//  

void Bresenham2D::initialize(int x_stop, int y_stop)
{
    // Update internal state
    x1 = 0;
    x2 = x_stop;
    y1 = 0;
    y2 = y_stop;

    delta_x_ = x2 - x1;
    // if x1 == x2, then it does not matter what we set here
    ix = (delta_x_ > 0) - (delta_x_ < 0);
    delta_x_ = static_cast<int>( std::abs(delta_x_) ) << 1;

    delta_y_ = y2 - y1;
    // if y1 == y2, then it does not matter what we set here
    iy = (delta_y_ > 0) - (delta_y_ < 0);
    delta_y_ = static_cast<int>( std::abs(delta_y_) ) << 1;

    dx_geq_dy_ = (delta_x_ >= delta_y_);

    // error may go below zero
    if (dx_geq_dy_)
        error_ = delta_y_ - (delta_x_ >> 1);
    else
        error_ = delta_x_ - (delta_y_ >> 1);

    initialized_ = true;
}

bool Bresenham2D::getNext(int& x_next, int& y_next)
{
    if (!initialized_)
        return false;

    if (dx_geq_dy_)
    {
        // As long as x1 != x2, we can go on
        if (x1 != x2)
        {
            if ((error_ >= 0) && (error_ || (ix > 0)))
            {
                error_ -= delta_x_;
                y1 += iy;
            }
            // else do nothing

            error_ += delta_y_;
            x1 += ix;

            // Assign values
            x_next = x1;
            y_next = y1;
        }
        else
        {
            initialized_ = false;
        }
    }
    else
    {
        // As long as y1 != y2, we can go on
        if (y1 != y2)
        {
            if ((error_ >= 0) && (error_ || (iy > 0)))
            {
                error_ -= delta_y_;
                x1 += ix;
            }
            // else do nothing

            error_ += delta_x_;
            y1 += iy;

            // Assign values
            x_next = x1;
            y_next = y1;
        }
        else
        {
            initialized_ = false;
        }
    }
    return initialized_;
}
