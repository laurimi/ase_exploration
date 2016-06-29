#ifndef BRESENHAM2D_H
#define BRESENHAM2D_H

/**
  * class Bresenham2D
  *
  */
// Simple Bresenham algorithm for 2D ray tracing
class Bresenham2D
{
public:
    Bresenham2D();
    // From (0,0) to x_stop, y_stop
    void initialize(int x_stop, int y_stop);
    bool getNext(int& x_next, int& y_next);

private:
    bool initialized_;

    // Internal state for the algorithm; updated for new query and when getting next
    int x1, y1, x2, y2; // Current location in trace
    int delta_x_, delta_y_;
    int error_;
    signed char ix, iy;
    bool dx_geq_dy_;
};

#endif // BRESENHAM2D_H
