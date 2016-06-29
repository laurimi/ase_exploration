#include "ExplorationPlanner/planner/ParticleSet.h"
#include <boost/random/uniform_01.hpp>
#include <cmath>
#include <sstream>

void ParticleSet::weightParticles(const std::vector<TrajectoryValueHandler>& h)
{
    assert( h.size() == getNumberOfParticles() );
    // Get min+max rewards
    double rmin = std::numeric_limits<double>::max();
    double rmax = -std::numeric_limits<double>::max();
    for ( auto it = h.begin(), iend = h.end(); it != iend; ++it)
    {
        rmin = std::min(rmin, it->getMinSumOfDiscRew() );
        rmax = std::max(rmax, it->getMaxSumOfDiscRew() );
    }

    double scaleFactor = (rmax - rmin);
    if (rmax == rmin)
        scaleFactor = 1;

    // Now we can scale the particles
    for ( unsigned int i = 0; i < getNumberOfParticles(); ++i )
    {
        double w = h[i].getWeightFactor(rmin, scaleFactor);
        particles_[i].setWeight( w * particles_[i].getWeight()  );
    }
    normalizeWeights();
}

void ParticleSet::resample(RNG& rng)
{
    if (getNumberOfParticles() <= 1)
        return;

    const double interval = getSumOfWeights() / getNumberOfParticles();
    boost::random::uniform_01<double> d;
    double target = interval * rng(d); //Initial target weight

    // Compute resampled vector
    std::vector<Particle> resampledParticles(getNumberOfParticles(), particles_.front() );
    double cumw = 0;
    unsigned int n = 0;
    for (auto it=particles_.begin(), iend=particles_.end(); it != iend; ++it)
    {
        cumw += it->getWeight();
        while (cumw > target)
        {
            resampledParticles[n++] = *it; // Postfix increment is crucial.
            target += interval;
        }
    }

    // Assign the new particles and normalize weights
    particles_ = resampledParticles;
    normalizeWeights();
}

Particle ParticleSet::getMaxWeightParticle() const
{
    Particle p;
    double w = -std::numeric_limits<double>::max();
    for (auto it = particles_.begin(), iend = particles_.end(); it != iend; ++it)
    {
        double cw = it->getWeight();
        if (w < cw)
        {
            w = cw;
            p = *it;
        }
    }
    return p;
}

double ParticleSet::getEffectiveNumberOfParticles() const
{
    double w_sq = 0.0;
    for (auto it = particles_.begin(), iend = particles_.end(); it != iend; ++it)
        w_sq += std::pow(it->getWeight(), 2);
    return (1.0/w_sq);
}

void ParticleSet::normalizeWeights()
{
    double w = getSumOfWeights();
    for (auto it = particles_.begin(), iend = particles_.end(); it != iend; ++it)
        it->setWeight( it->getWeight() / w  );
}

double ParticleSet::getSumOfWeights() const
{
    double w = 0.0;
    for (auto it = particles_.begin(), iend = particles_.end(); it != iend; ++it)
        w += it->getWeight();
    return w;
}

std::string ParticleSet::Print() const
{
    std::stringstream ss;
    const unsigned int sz = particles_.size();
    ss << "Particle set with " << sz << " particles\n";
    for (unsigned int i = 0; i < sz; ++i)
        ss << "Particle " << i << ":\n" << particles_[i].Print() << "\n";
    return ss.str();
}
