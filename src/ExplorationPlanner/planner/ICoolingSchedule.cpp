#include "ExplorationPlanner/planner/ICoolingSchedule.h"
#include <iostream>

std::ostream &operator<<(std::ostream &os, const ICoolingSchedule& s)
{
    os << s.Print();
    return os;
}
