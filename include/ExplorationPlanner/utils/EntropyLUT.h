#ifndef ENTROPYLUT_H
#define ENTROPYLUT_H
#include <cstdint>
#include <iostream>
class EntropyLUT
{
public:
    double get(int8_t i) const
    {
        if ((i < 0) || (i > 100))
            std::cout << "EntropyLUT::get(i) -- ERROR: i = " << static_cast<int>(i) << " is not valid, expect errors!\n";

        return entropy_[i];
    }

private:
    static double entropy_[101];
};

#endif // ENTROPYLUT_H
