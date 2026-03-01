// RobotState.cpp

#include "vex.h"
#include "RobotState.hpp"
#include "Lock.hpp"
#include "OdometryTracker.hpp"


using namespace vex;


RobotState::RobotState(IMU& imu, OdometryTracker& odom) : inert(imu), odom(odom) {}



double RobotState::getTrueHeading() {
    Lock trueHeadingLock(trueHeadingMutex);
    trueHeading = inert.getTrueHeading();
    return trueHeading;
}

double RobotState::getRotation() {
    return inert.getRotation();
}

void RobotState::setRotation(double rotation) {
    inert.setRotation(rotation);
}

/**
 * @brief Calculates the smallest angle difference between two angles in degrees.
 * 
 * This function computes the difference between a target angle (`targ`) and a current angle (`curr`),
 * taking into account the wrapping behavior of angles. The result will always be between -180 and 180 degrees,
 * representing the shortest angular distance.
 * 
 * @param ref The reference angle in degrees.
 * @param targ The target angle in degrees.
 * @return double The smallest angular difference between the two angles, in degrees.
 */

double RobotState::getAngleDifference(double ref, double targ) {
    if (fabs(targ-ref) <= 180) {
        return targ - ref;
    } else {
        if (targ-ref > 0) {
            return targ-ref-360;
        } else {
            return targ-ref+360;
        }
    }
}

double RobotState::getTrueYaw() {
    Lock trueYawLock(trueYawMutex);
    return trueYaw;
}

double RobotState::getTruePitch() {
    Lock truePitchLock(truePitchMutex);
    return truePitch;
}

double RobotState::getTrueRoll() {
    Lock trueRollLock(trueRollMutex);
    return trueRoll;
}

double RobotState::getSpd() {
    Lock spdLock(spdMutex);
    return spd;
}

double RobotState::getTurnSpd() {
    Lock turnSpdLock(turnSpdMutex);
    return turnSpd;
}

double RobotState::getXPos() {
    Lock xPosLock(xPosMutex);
    xPos = 
    return xPos;
}

double RobotState::getYPos() {
    Lock yPosLock(yPosMutex);
    return yPos;
}


void RobotState::setBallPathState(BallPathState state) {
    Lock lock(ballPathStateMutex);
    ballPathState = state;
}

BallPathState RobotState::getBallPathState() {
    Lock lock(ballPathStateMutex);
    return ballPathState;
}