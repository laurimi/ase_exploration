#ifndef PARTICLESET_H
#define PARTICLESET_H

#include "Particle.h"
#include "ExplorationPlanner/utils/RNG.h"
#include "ExplorationPlanner/trajectory/TrajectoryValueHandler.h"
#include <vector>

/**
  * class ParticleSet
  *
  */

class ParticleSet
{
public:
    // Constructors/Destructors
    //
    ParticleSet()
    {

    }

    ParticleSet(unsigned int n, const Particle& p)
        : particles_(n, p)
    {
        normalizeWeights();
    }

    Particle& operator[] ( unsigned int i )
    {
        if ( i >= particles_.size() )
            throw std::out_of_range("Requested particle index out of range");
        return particles_[i];
    }

    /**
   */
    void resample(RNG& rng);

    /**
   */
    void normalizeWeights();

    /**
   * @return double
   */
    double getEffectiveNumberOfParticles() const;

    /**
   * @return unsigned int
   */
    unsigned int getNumberOfParticles() const {
        return particles_.size();    }

    std::string Print() const;

    void weightParticles(const std::vector<TrajectoryValueHandler>& h);

    Particle getMaxWeightParticle() const;

private:
    /**
   * @return double
   */
    double getSumOfWeights() const;

    // Private attributes
    //
    std::vector<Particle> particles_;
};

#endif // PARTICLESET_H
