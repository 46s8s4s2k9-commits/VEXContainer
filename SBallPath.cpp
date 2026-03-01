// SBallPath.cpp

#include "vex.h"
#include "Lock.hpp"
#include "SBallPath.hpp"

extern vex::digital_out PnFDescore;
extern vex::digital_out PnIntake;

extern bool isIntakeUp;


using namespace vex;

SBallPath::SBallPath(motor& intake, motor& outtake, motor& end, RobotState& state) : intake(intake), outtake(outtake), end(end), state(state) {}

void SBallPath::setIntakeEndSpd(double spd) {
    intakeEndSpd = spd;
}

void SBallPath::storeBalls(double in, double out) {
    state.setBallPathState(BallPathState::Store);
    intake.spin(forward, in, pct);
    outtake.spin(forward, out, pct);
    end.spin(forward, intakeEndSpd, pct);
}

/*
void SBallPath::storeBalls() {
    storeBalls(100, 100);
}
*/

void SBallPath::lowOut(double in, double out) {
    isIntakeUp = true;
    PnIntake.set(true);
    state.setBallPathState(BallPathState::Low);
    intake.spin(forward, -in, pct);
    outtake.spin(forward, -out, pct);
    end.spin(forward, 20, pct);
}

/*
void SBallPath::lowOut() {
    lowOut(100, 100);
}
*/

void SBallPath::highOut(double in, double out, double e) {
    PnFDescore.set(true);
    isFDescoreUp = true;
    state.setBallPathState(BallPathState::High);
    intake.spin(fwd, in, pct);
    outtake.spin(fwd, out, pct);
    end.spin(fwd, e, pct);
}

void SBallPath::highDelay(double in, double out, double e) {
    state.setBallPathState(BallPathState::HighDel);
    intake.spin(fwd, in, pct);
    outtake.spin(fwd, -out, pct);
    end.spin(fwd, e, pct);
}
/*
void SBallPath::highOut() {
    highOut(100, 100, 100);
}
*/

void SBallPath::midOut(double in, double out, double e) {
    state.setBallPathState(BallPathState::Mid);
    intake.spin(fwd, in, pct);
    outtake.spin(fwd, out, pct);
    end.spin(fwd, -e, pct);
}

void SBallPath::highRev(double in, double out, double e) {
    state.setBallPathState(BallPathState::HighRev);
    intake.spin(fwd, in, pct);
    outtake.spin(fwd, -out, pct);
    end.spin(fwd, -e, pct);
}

/*
void SBallPath::midOut() {
    midOut(100, 100, 100);
}
*/

void SBallPath::stop() {
    state.setBallPathState(BallPathState::None);
    intake.stop();
    outtake.stop();
    end.stop();
}

double SBallPath::getIntakeSpd() {
    return intake.velocity(pct);
}

double SBallPath::getOuttakeSpd() {
    return outtake.velocity(pct);
}

double SBallPath::getEndSpd() {
    return end.velocity(pct);
}