// SBallPath.hpp

#ifndef CONTROLLERSCREEN_HPP
#define CONTROLLERSCREEN_HPP

#include "vex.h"
#include "RobotState.hpp"

extern vex::brain Brain;

class ControllerScreen {
    public:
        ControllerScreen(vex::controller Controller1);
        void refresh();

        void setPn(double num);
        void setTeam(bool teamIsBlue);
        int timer;
    private:
        vex::controller Controller1;
        double pn;
        bool teamIsBlue;
};

#endif // CONTROLLERSCREEN_HPP