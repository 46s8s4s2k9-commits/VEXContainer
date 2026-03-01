// IMU.cpp

#include "vex.h"
#include "IMU.hpp"
#include "Lock.hpp"


using namespace vex;

IMU::IMU(vex::inertial& imu) : imu(imu) {}

double IMU::getTrueHeading() {
    double heading = imu.rotation(degrees);
    
    return fmod(heading, 360);
}

double IMU::getRotation() {
    return imu.rotation(degrees);
}

void IMU::calibrate() {
    imu.calibrate();
}

bool IMU::isCalibrating() {
    return imu.isCalibrating();
}

void IMU::resetAll() {
    imu.resetHeading();
    imu.resetRotation();
}

bool IMU::isAtRest() {
    if (fabs(imu.gyroRate(xaxis, dps)) < 5) {
        if (fabs(imu.gyroRate(yaxis,dps)) < 5) {
            if (fabs(imu.gyroRate(zaxis,dps)) < 5) {
                if (fabs(imu.orientation(pitch, degrees)) < 5.0) {
                    if (fabs(imu.orientation(roll, degrees)) < 5.0) {
                        if (fabs(imu.acceleration(xaxis)) < 0.1) {
                            if (fabs(imu.acceleration(yaxis)) < 0.1) {
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
}

void IMU::setHeading(double h) {
    imu.setHeading(h, degrees);
}

void IMU::setRotation(double r) {
    imu.setRotation(r, degrees);
}

void IMU::setIMUOrientation(Axis imuXPos, Axis imuXNeg, Axis imuYPos, Axis imuYNeg, Axis imuZPos, Axis imuZNeg) {
    xPositive = imuXPos;
    xNegative = imuXNeg;
    yPositive = imuYPos;
    yNegative = imuYNeg;
    zPositive = imuZPos;
    zNegative = imuZNeg;
}