// SixWheelDrive.cpp

#include "SixWheelDrive.hpp"
#include "vex.h"

using namespace vex;

SixWheelDrive::SixWheelDrive(motor& lf, motor& lm, motor& lb, motor& rf, motor& rm, motor& rb) 
    : lf(lf), lm(lm), lb(lb), rf(rf), rm(rm), rb(rb), 
      initZeroTime(0), zeroTimeUsed(false), breakMSec(500), 
      zeroTimeLapse(0), driveSpd(2), driveSpdThree(1.0), 
      driveSpdTwo(0.75), driveSpdOne(0.5), lftCorr(1.0), 
      rghtCorr(1.0), joystickDeadzone(5) {}




// Function controlling left motor group only
void SixWheelDrive::lft_drive(double spd) {
    lf.spin(forward, spd, pct);
    lm.spin(forward, spd, pct);
    lb.spin(forward, spd, pct);
} 

// Function controlling right motor group only
void SixWheelDrive::rght_drive(double spd) {
    rf.spin(forward, spd, pct);
    rm.spin(forward, spd, pct);
    rb.spin(forward, spd, pct);
}

void SixWheelDrive::lrDrive(double lspd, double rspd) {
    lft_drive(lspd);
    rght_drive(rspd);
}


void SixWheelDrive::basic_drive(double leftSpd, double rightSpd) {
    // Apply deadzone
    if (fabs(leftSpd) < joystickDeadzone) leftSpd = 0;
    if (fabs(rightSpd) < joystickDeadzone) rightSpd = 0;

    // Select speed multiplier
    double speedMult = (driveSpd == 3) ? driveSpdThree :
                        (driveSpd == 2) ? driveSpdTwo :
                                        driveSpdOne;

    // Apply multiplier
    double lft = leftSpd * speedMult;
    double rght = rightSpd * speedMult;

    // Apply motor correction
    double lftCorrected = lft * lftCorr;
    double rghtCorrected = rght * rghtCorr;

    // If either exceeds 100%, scale BOTH down proportionally
    double maxAbs = fmax(fabs(lftCorrected), fabs(rghtCorrected));
    
    if (maxAbs > 100) {
        double scale = 100.0 / maxAbs;
        lftCorrected *= scale;
        rghtCorrected *= scale;
    }

    // Final safety clamp (shouldn't trigger if above logic works)
    if (lftCorrected > 100) lftCorrected = 100;
    if (lftCorrected < -100) lftCorrected = -100;
    if (rghtCorrected > 100) rghtCorrected = 100;
    if (rghtCorrected < -100) rghtCorrected = -100;

    // Drive motors
    lft_drive(lftCorrected);
    rght_drive(rghtCorrected);
}


// Function to reset all encoders in the drivetrain
void SixWheelDrive::resetEnc(double lft, double rght) { // in degrees
    lf.setPosition(lft, degrees);
    lm.setPosition(lft, degrees);
    lb.setPosition(lft, degrees);

    rf.setPosition(rght, degrees);
    rm.setPosition(rght, degrees);
    rb.setPosition(rght, degrees);
}

double SixWheelDrive::getLeftEncoder() {
    double lftEnc = (lf.position(degrees) + lm.position(degrees) + lb.position(degrees))/3.0;
    return lftEnc;
}

double SixWheelDrive::getRightEncoder() {
    double rghtEnc = (rf.position(degrees) + rm.position(degrees) + rb.position(degrees))/3.0;
    return rghtEnc;
}

double SixWheelDrive::getLeftVelocity() {
    double lftVel = (lf.velocity(percent) + lm.velocity(percent) + lb.velocity(percent))/3.0;
    return lftVel;
}

double SixWheelDrive::getRightVelocity() {
    double rghtVel = (rf.velocity(percent) + rm.velocity(percent) + rb.velocity(percent))/3.0;
    return rghtVel;
}

double SixWheelDrive::getLeftTemp() {
    return (lf.temperature(percent) + lm.temperature(percent) + lb.temperature(percent))/3.0;
}

double SixWheelDrive::getRightTemp() {
    return (rf.temperature(percent) + rm.temperature(percent) + rb.temperature(percent))/3.0;
}

void SixWheelDrive::turnSpinFor(double deg) {
    lf.spinFor(forward, deg, degrees, false);
    lm.spinFor(forward, deg, degrees, false);
    lb.spinFor(forward, deg, degrees, false);
    rf.spinFor(forward, -deg, degrees, false);
    rm.spinFor(forward, -deg, degrees, false);
    rb.spinFor(forward, -deg, degrees, true);
}

void SixWheelDrive::allSpinFor(double deg) {
    lf.spinFor(forward, deg, degrees, false);
    lm.spinFor(forward, deg, degrees, false);
    lb.spinFor(forward, deg, degrees, false);
    rf.spinFor(forward, deg, degrees, false);
    rm.spinFor(forward, deg, degrees, false);
    rb.spinFor(forward, deg, degrees, true);
}

void SixWheelDrive::allStop() {
    lf.stop();
    lm.stop();
    lb.stop();
    rf.stop();
    rm.stop();
    rb.stop();
}

double SixWheelDrive::degToCm(double degree) {
    return (degree / 360.0) * gearRatio * wheelCirc;
}

double SixWheelDrive::cmToDeg(double cm) {
    return ((cm / wheelCirc) / gearRatio) * 360.0;
}


void SixWheelDrive::setCorrections(double lftCorr, double rghtCorr) {
    this->lftCorr = lftCorr;
    this->rghtCorr = rghtCorr;
}

void SixWheelDrive::setDriveSpeed(int speedLevel) {
    if (speedLevel < 1) speedLevel = 1;
    if (speedLevel > 3) speedLevel = 3;
    driveSpd = speedLevel;
}

void SixWheelDrive::setCoast() {
    lf.setStopping(coast);
    lm.setStopping(coast);
    lb.setStopping(coast);
    rf.setStopping(coast);
    rm.setStopping(coast);
    rb.setStopping(coast);
}

void SixWheelDrive::setHold() {
    lf.setStopping(hold);
    lm.setStopping(hold);
    lb.setStopping(hold);
    rf.setStopping(hold);
    rm.setStopping(hold);
    rb.setStopping(hold);
}

void SixWheelDrive::setBrake() {
    lf.setStopping(brake);
    lm.setStopping(brake);
    lb.setStopping(brake);
    rf.setStopping(brake);
    rm.setStopping(brake);
    rb.setStopping(brake);
}
