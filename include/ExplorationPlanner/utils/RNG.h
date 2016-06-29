#ifndef RNG_H
#define RNG_H

#include <boost/random/mersenne_twister.hpp>
#include <ctime>
#include <vector>
#include <omp.h>

/**
  * class RNG
  *
  */

class RNG
{
public:
    typedef boost::random::mt19937 Engine;

    RNG() : engines_()
    {
        const int threads = std::max(1, omp_get_max_threads());
        // Seed first engine with current time, others by using random numbers from previous generator
        // not ideal but oh well...
        engines_.push_back(Engine(std::time(0)));
        for(int seed = 1; seed < threads; ++seed)
        {
            engines_.push_back(Engine(engines_[seed-1]));
        }
    }

    void setSeed(unsigned int seed)
    {
        if (engines_.empty())
            return;

        engines_[0].seed(seed);
        for (int i = 1; i < engines_.size(); ++i)
            engines_[i].seed( engines_[i-1]() );
    }

    template <class DistributionType>
    typename DistributionType::result_type operator()(DistributionType& d)
    {
        return d(engines_[omp_get_thread_num()]);
    }

private:
    std::vector<Engine> engines_;
};

#endif // RNG_H
