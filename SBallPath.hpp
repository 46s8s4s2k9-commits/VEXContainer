// SBallPath.hpp

#ifndef SBALLPATH_HPP
#define SBALLPATH_HPP

#include "vex.h"
#include "RobotState.hpp"

extern bool isFDescoreUp;
extern vex::brain Brain;

class SBallPath {
    private:
        vex::motor& intake;
        vex::motor& outtake;
        vex::motor& end;
        double intakeEndSpd = 15;

        RobotState& state;

    public:


        SBallPath(vex::motor& intake, vex::motor& outtake, vex::motor& end, RobotState& state);

        void storeBalls(double in = 100, double out = 100);
        // void storeBalls();

        void lowOut(double in = 50, double out = 100);
        // void lowOut();

        void highOut(double in = 100, double out = 100, double end = 100);
        // void highOut();

        void midOut(double in = 100, double out = 100, double end = 30);
        // void midOut();

        void highRev(double in = 100, double out = 5, double end = 100);

        void highDelay(double in = 100, double out = 40, double end = 100);

        void stop();

        double getIntakeSpd();
        double getOuttakeSpd();
        double getEndSpd();

        void setIntakeEndSpd(double spd);
        
};









#endif // SBALLPATH_HPP