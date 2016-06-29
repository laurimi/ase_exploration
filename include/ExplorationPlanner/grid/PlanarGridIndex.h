#ifndef PLANARGRIDINDEX_H
#define PLANARGRIDINDEX_H
#include <boost/functional/hash.hpp>
/**
  * class PlanarGridIndex
  *
  */

class PlanarGridIndex
{
public:

    // Constructors/Destructors
    //
    PlanarGridIndex(int x, int y) : x_(x), y_(y) { }

    // Public attributes
    //
    friend std::size_t hash_value(const PlanarGridIndex& m)
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, m.x_);
        boost::hash_combine(seed, m.y_);
        return seed;
    }

private:

    // Private attributes
    //

    int x_;
    int y_;

public:
    // Private attribute accessor methods
    //
    /**
     * Set the value of x_
     * @param x the new value of x_
     */
    void setX(double x) {
        x_ = x;
    }

    /**
     * Set the value of y_
     * @param x the new value of y_
     */
    void setY(double y) {
        y_ = y;
    }

    /**
   * Get the value of x_
   * @return the value of x_
   */
    int getX() const   {
        return x_;
    }

    /**
   * Get the value of y_
   * @return the value of y_
   */
    int getY() const  {
        return y_;
    }

};

// Used for hashing
inline bool operator==(const PlanarGridIndex& ma, const PlanarGridIndex& mb)
{
    return ( (ma.getX() == mb.getX()) && (ma.getY() == mb.getY()) );
}

// Used for hashing
inline bool operator!=(const PlanarGridIndex& ma, const PlanarGridIndex& mb)
{
    return !( ma == mb);
}


#endif // PLANARGRIDINDEX_H
