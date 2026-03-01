// Auton.hpp

#ifndef AUTON_HPP
#define AUTON_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "RobotState.hpp"
#include "SixWheelDrive.hpp"
#include "OdometryTracker.hpp"
#include "ControlUnit.hpp"
#include "Lock.hpp"
#include "vex.h"


extern vex::brain Brain;

struct TransitionWPT {
    double angle;
    double leg;
    double safetyDist;

    TransitionWPT(double angle, double leg, double safetyDist) : angle(angle), leg(leg), safetyDist(safetyDist) {}
    TransitionWPT(double angle, double leg) : angle(angle), leg(leg), safetyDist(0) {}
    TransitionWPT() : angle(0), leg(0), safetyDist(0) {}
};

struct AutoTuneFeedback {
    double o1;
    double o2;
    bool atO2;
};

enum class AutonState {
    Forward,
    Reverse,
    TurnRight,
    SingTurnRight,
    TurnLeft,
    SingTurnLeft,
    None
};

class Auton {
public:
    Auton(RobotState& state, SixWheelDrive& driveBase, OdometryTracker& odometry, ControlUnit& control);
    void setPIDLimit(double lim);
    void setPIDTimeout(double ms);
    void setAutonCorrections(double lftCorr, double rghtCorr);
    double degToCm(double degree);
    double cmToDeg(double cm);
    void resetAction();
    double dynamicKI(double maxKi, double minKi, double maxError, double currError);
    void PIDdriveForward(double dist, bool timeOut = true);
    void PIDturnRight(double deg);
    void PIDsingTurnR(double deg);
    void PIDturnLeft(double deg);
    void PIDsingTurnL(double deg);
    double PID(double error, double kp, double ki, double kd);
    double getCurrentError();
    double getCurrentTarget();
    double getTurnError();
    double getTurnTarget();
    double getDistanceRemaining();
    double getTurnDegreesRemaining();
    void setPIDConstants(double p, double i, double d);
    void setTurnPIDConstants(double p, double i, double d);
    bool isPIDOver();
    AutonState getAutonState();
    void setPIDDriveFwdCorr(double index);
    void setTrustInert(bool trust);
    void setUseControl(bool control);

private:
    RobotState& state;
    SixWheelDrive& driveBase;
    OdometryTracker& odometry;
    ControlUnit& control;

    double pidLimit = 80;
    double pidTimeout = 5000;
    bool pidOver = false;

    bool useControl = false;

    double pidDriveFwdCorr = 1.0;

    bool trustInert = true;
    vex::mutex trustInertMutex;

    vex::mutex pidOverMutex;
    AutonState autonState;
    vex::mutex autonStateMutex;
    double autonLftCorr;
    double autonRghtCorr;
    double kP;
    double kI;
    double kD;
    double turnKp;
    double turnKi;
    double turnKd;
    double initX;
    double initY;

    int lastTime;
    int currTime;

    double target;
    double initError;
    double lastError;
    vex::mutex lastErrorMutex;

    double turnTarget;
    double turnInitError;
    double turnLastError;

    double proportional;
    double integral;
    double derivative;

    void drivePct(double leftSpd, double rightSpd);
    void transition2Leg(double leg1, TransitionWPT pt2);
    void transition3Leg(double leg1, TransitionWPT pt2, TransitionWPT pt3);
};

#endif // AUTON_HPP
