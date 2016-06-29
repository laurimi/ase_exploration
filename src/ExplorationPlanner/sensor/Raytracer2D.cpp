#include "ExplorationPlanner/sensor/Raytracer2D.h"
#include "ExplorationPlanner/sensor/Bresenham2D.h"

// Implementation by projecting all rays to start at origin + cache results
void Raytracer2D::initialize(int x_start, int y_start, int x_stop, int y_stop)
{
    // Assign start pt
    xstart_ = x_start;
    ystart_ = y_start;

    // Transform to start from zero
    int xt = (x_stop - x_start);
    int yt = (y_stop - y_start);

    PlanarGridIndex idx(xt, yt);
    RayTracePtMap::iterator itc = rt_cache_.find(idx);
    if ( itc == rt_cache_.end() )
    {
        // Precompute to cache, then set the pointer
        Bresenham2D br;
        br.initialize(xt, yt);
        int xn, yn;
        RayTracePtVec v;
        while (br.getNext(xn, yn))
            v.push_back( RayTracePt(xn, yn) );

        std::pair<RayTracePtMap::iterator, bool> in = rt_cache_.insert( std::make_pair(idx, v) );
        itc = in.first;
    }

    // Set iterators to point to start and end of appropriate raytrace point vector
    ipt_ = itc->second.begin();
    ipt_end_ = itc->second.end();
}

bool Raytracer2D::getNext(int& x_next, int& y_next)
{
    if (ipt_ == ipt_end_)
        return false;

    x_next = xstart_ + ipt_->first;
    y_next = ystart_ + ipt_->second;
    ++ipt_;
    return true;
}
