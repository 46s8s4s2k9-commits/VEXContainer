// SixWheelDrive.hpp

#ifndef SIXWHEELDRIVE_HPP
#define SIXWHEELDRIVE_HPP


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "vex.h"


extern vex::brain Brain;

class SixWheelDrive {
    public:
        SixWheelDrive(vex::motor& lf, vex::motor& lm, vex::motor& lb, vex::motor& rf, vex::motor& rm, vex::motor& rb);
        void lft_drive(double spd);
        void rght_drive(double spd);
        void lrDrive(double lspd, double rspd);
        void basic_drive(double leftSpd, double rightSpd);
        void resetEnc(double lft, double rght);
        double getLeftEncoder();
        double getRightEncoder();
        double getLeftVelocity();
        double getRightVelocity();
        double getLeftTemp();
        double getRightTemp();
        void turnSpinFor(double deg);
        void allSpinFor(double deg);
        void allStop();
        double degToCm(double degree);
        double cmToDeg(double cm);
        void setCorrections(double lftCorr, double rghtCorr);
        void setDriveSpeed(int speedLevel);
        void setCoast();
        void setHold();
        void setBrake();

        double lftCorr;
        double rghtCorr;
        int driveSpd;
        double driveSpdThree;
        double driveSpdTwo;
        double driveSpdOne;
        double joystickDeadzone;
        double baseWidth;
        double wheelDiameter;
        double wheelCirc;
        double gearRatio;
        double initZeroTime;
        bool zeroTimeUsed;
        double breakMSec;
        double zeroTimeLapse;
        bool isHolding = false;

    private:
        vex::motor& lf;
        vex::motor& lm;
        vex::motor& lb;
        vex::motor& rf;
        vex::motor& rm;
        vex::motor& rb;
};

#endif // SIXWHEELDRIVE_HPP
