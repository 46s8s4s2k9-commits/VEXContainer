// IMU.hpp

#ifndef IMU_HPP
#define IMU_HPP

#include "vex.h"

enum class Axis {
    XPos,
    YPos,
    XNeg,
    YNeg,
    ZPos,
    ZNeg
};


class IMU {
    public:
        
        vex::inertial& imu;

        double xPos;
        vex::mutex xPosMutex;
        double yPos;
        vex::mutex yPosMutex;
        double trueHeading;
        vex::mutex trueHeadingMutex;

        Axis xPositive; // Inertial xpositive, not true xpositive, so xPositive = RobotAxis::XNeg; means Robot's x negative axis is imu's x positive imu pointing backwards
        Axis xNegative;
        Axis yPositive;
        Axis yNegative;
        Axis zPositive;
        Axis zNegative;

        IMU(vex::inertial& imu);

        double getTrueHeading();
        double getRotation();

        void calibrate();
        bool isCalibrating();

        void resetAll();

        bool isAtRest();

        void setHeading(double h);
        void setRotation(double r);

        void setIMUOrientation(Axis imuXPos, Axis imuXNeg, Axis imuYPos, Axis imuYNeg, Axis imuZPos, Axis imuZNeg);

};

#endif //IMU_HPP