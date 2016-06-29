#ifndef PLANARGRIDCONTAINER_H
#define PLANARGRIDCONTAINER_H
#include "PlanarGridIndex.h"
#include <boost/unordered_map.hpp>

/**
  * class PlanarGridContainer
  *
  */

// A sparse grid map container for arbitrary data type T.
template<class T>
class PlanarGridContainer
{
public:
    typedef boost::unordered_map<PlanarGridIndex, T, boost::hash<PlanarGridIndex> > Container;

    typedef typename Container::const_iterator ContainerConstIter;
    typedef typename Container::iterator ContainerIter;

    /**
   * @return T  const reference to mapped data type
   * @param  idx index to get data from
   */
    const T& getGridValue(const PlanarGridIndex& idx) const
    {
        // Throws out_of_range if idx does not match any key in M_:
        // see: http://www.cplusplus.com/reference/unordered_map/unordered_map/at/
        return M_.at(idx);
    }

    /**
   * @param  idx
   * @param  v
   */
    void setGridValue(const PlanarGridIndex& idx, const T& v)
    {
        // default constructor needed for T:
        // see http://www.cplusplus.com/reference/unordered_map/unordered_map/operator[]/
        M_[idx] = v;

        // Only works if value is *inserted*, does not modify existing:
        // see http://www.cplusplus.com/reference/unordered_map/unordered_map/insert/
        //M_.insert( std::make_pair(idx, v) );
    }

    /**
   * @param  idx
   * @param  v
   */
    bool isInContainer(const PlanarGridIndex& idx) const
    {
        return ( M_.find(idx) != M_.end()  );
    }

    unsigned int size() const
    {
        return M_.size();
    }

    bool empty() const
    {
        return M_.empty();
    }

    /**
   * @return ContainerType::const_iterator
   * @param  idx
   */
    ContainerConstIter find(const PlanarGridIndex& idx) const
    {
        return M_.find(idx);
    }

    /**
   * @return ContainerType::const_iterator
   * @param  idx
   */
    ContainerConstIter begin() const
    {
        return M_.begin();
    }

    /**
   * @return ContainerType::const_iterator
   * @param  idx
   */
    ContainerConstIter end() const
    {
        return M_.end();
    }

    // Gets minimum and maximum values for the maps indices
    bool getMapExtents(int& x_min, int& x_max, int& y_min, int& y_max) const
    {
        if (M_.empty())
            return false;

        x_min = M_.cbegin()->first.getX();
        x_max = M_.cbegin()->first.getX();
        y_min = M_.cbegin()->first.getY();
        y_max = M_.cbegin()->first.getY();
        for (ContainerConstIter it = M_.cbegin(), iend = M_.cend(); it != iend; ++it)
        {
            x_min = std::min(x_min, it->first.getX());
            x_max = std::max(x_max, it->first.getX());
            y_min = std::min(y_min, it->first.getY());
            y_max = std::max(y_max, it->first.getY());
        }
        return true;
    }



private:
    // Private attributes
    //
    Container M_;
};

#endif // PLANARGRIDCONTAINER_H
