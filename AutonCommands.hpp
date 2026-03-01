// AutonCommands.hpp

#ifndef AUTONCOMMANDS_HPP
#define AUTONCOMMANDS_HPP

#include "Auton.hpp"
#include "OdometryTracker.hpp"
#include "PneumaticsTracker.hpp"
#include "RobotState.hpp"
#include "SBallPath.hpp"
#include "vex.h"


extern vex::brain Brain;



class AutonCommands {
    public:
        AutonCommands(Auton& a, OdometryTracker& o, RobotState& state, SBallPath& ballPath);
        void moveTo(double targetX, double targetY, bool precise = true, double finalHeading = -999);
        void intakeBalls(int durationMs);
        void scoreHighGoal(int durationMs);
        void scoreMidGoal(int durationMs);
        void turnToHeading(double targetHeading, bool precise = true);
        void singTurnToHeading(double targetHeading, bool isFwd = true, bool precise = true);

    private:
        Auton& auton;
        OdometryTracker& odometry;
        RobotState& state;
        SBallPath& ballPath;
        int turnToleranceDeg = 2;
};

#endif // AUTONCOMMANDS_HPP
