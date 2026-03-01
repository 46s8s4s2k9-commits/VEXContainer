// PurePursuit.hpp 

#ifndef PUREPURSUIT_HPP
#define PUREPURSUIT_HPP

#include "vex.h"
#include <vector>
#include "Lock.hpp"
#include "SixWheelDrive.hpp"

struct Coord {
    double x;
    double y;
    bool hasHeading;
    double heading;

    Coord() : hasHeading(false) {}
    Coord(double x_, double y_) : x(x_), y(y_), hasHeading(false) {}
    Coord(double x_, double y_, double heading_) : x(x_), y(y_), hasHeading(true), heading(heading_) {}
};

struct PPFeedBack {
    Coord coord;
    int lastFoundIndex;
    double turnError;
    bool intersectFound;
};

class PurePursuit {
    public:
        PurePursuit(SixWheelDrive& driveBase);
        void followPath(std::vector<Coord> path); 
    private:
        double degSin(double deg);
        double degaTan(double in);
        SixWheelDrive& driveBase;
        RobotState& state;
        int signOf(double x);
        double distBetween(Coord a, Coord b);
        double getAngleDifference(double ref, double targ);
        PPFeedBack PPStep(std::vector<Coord> path, Coord currState, double lookAheadDist, int LFoundInd);
        double calcTurnVel(double linearVel, double lookAheadDist, double degError);
        double calcTurnError(Coord currState, Coord targetPos);
};

#endif // PUREPURSUIT_HPP