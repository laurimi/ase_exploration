#ifndef LOGODDSLUT_H
#define LOGODDSLUT_H
#include <iostream>
#include <cstdint>

class LogOddsLUT
{
public:
    double get(int8_t i) const
    {
        if ((i < 0) || (i > 100))
            std::cout << "LogOddsLUT::get(i) -- ERROR: i = " << i << " is not valid, expect errors!\n";
        return log_odds_[i];
    }

private:
    static double log_odds_[101];
};


#endif // LOGODDSLUT_H
