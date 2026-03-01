// RobotState.hpp

#ifndef ROBOTSTATE_HPP
#define ROBOTSTATE_HPP

#include "vex.h"
#include "Lock.hpp"
#include "IMU.hpp"
#include "OdometryTracker.hpp"



enum class BallPathState {
    Store,
    Low,
    Mid,
    High,
    HighRev,
    HighDel,
    None
};



class RobotState {
    public:
        int32_t trueTimeCorr = 0;

        IMU& inert;
        OdometryTracker& odom;

        double trueHeading;
        vex::mutex trueHeadingMutex;
        double trueYaw;
        vex::mutex trueYawMutex;
        double truePitch;
        vex::mutex truePitchMutex;
        double trueRoll;
        vex::mutex trueRollMutex;
        double spd;
        vex::mutex spdMutex;
        double turnSpd;
        vex::mutex turnSpdMutex;

        double xPos;
        vex::mutex xPosMutex;
        double yPos;
        vex::mutex yPosMutex;

        BallPathState ballPathState;
        vex::mutex ballPathStateMutex;

        RobotState(IMU& imu, OdometryTracker& odom);

        double getTrueHeading();
        double getTrueYaw();
        double getTruePitch();
        double getTrueRoll();
        double getSpd();
        double getTurnSpd();
        double getRotation();

        void setRotation(double rotation);

        double getAngleDifference(double ref, double curr);
        
        double getXPos();
        double getYPos();

        void setBallPathState(BallPathState state);

        BallPathState getBallPathState();
};

#endif // ROBOTSTATE_HPP
