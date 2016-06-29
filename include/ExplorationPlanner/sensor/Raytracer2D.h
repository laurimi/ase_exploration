#ifndef RAYTRACER2D_H
#define RAYTRACER2D_H
#include "ExplorationPlanner/grid/PlanarGridIndex.h"
#include <boost/unordered_map.hpp>

class Raytracer2D
{
public:
    void initialize(int x_start, int y_start, int x_stop, int y_stop);
    bool getNext(int& x_next, int& y_next);

    unsigned int cacheSize() const {
        return rt_cache_.size();
    }

private:
    typedef std::pair<int, int> RayTracePt;
    typedef std::vector<RayTracePt> RayTracePtVec;
    typedef boost::unordered_map<PlanarGridIndex, RayTracePtVec, boost::hash<PlanarGridIndex> > RayTracePtMap;
    RayTracePtMap rt_cache_; // store all rays projected in memory for later lookup

    RayTracePtVec::const_iterator ipt_; // points to current pt in raytrace
    RayTracePtVec::const_iterator ipt_end_; // points to end pt of current raytrace
    int xstart_; // remember where we started the trace
    int ystart_;
};

#endif /* RAYTRACER_H */
