// PreAutonState.hpp

#ifndef PREAUTONSTATE_HPP
#define PREAUTONSTATE_HPP

#include "vex.h"

class PreAutonState {
    public:
        bool inertCal = false;
        bool duringCal = false;
        bool isScreenLocked = false;
        bool isAtRest = false;
        const int updateLoop = 4;
        int currLoop = 0;
        double heading = 0;
};

#endif //PREAUTONSTATE_HPP