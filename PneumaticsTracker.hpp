// PneumaticsTracker.hpp

#ifndef PNEUMATICSTRACKER_HPP
#define PNEUMATICSTRACKER_HPP

#include "vex.h"
extern vex::brain Brain;

class PneumaticsTracker {
public:
    PneumaticsTracker();
    void setMax(double max);
    void use(double units = 1.0);
    double remaining();
    void reset();
    double maxUses;
    double currentUses;

};

#endif // PNEUMATICSTRACKER_HPP
