// OdometryTracker.cpp

#include "OdometryTracker.hpp"
#include <cmath>
#include "Lock.hpp"

using namespace vex;

OdometryTracker::OdometryTracker(SixWheelDrive& driveBase, IMU& inert) : driveBase(driveBase), currentX(0), currentY(0), 
                                     currentHeading(0), lastLeftPos(0), lastRightPos(0), inert(inert) {}


void OdometryTracker::reset(double x, double y, double heading, double leftPos, double rightPos) {
    {
        Lock lockX(currentXMutex);
        Lock lockY(currentYMutex);
        Lock lock(currentHeadingMutex);
        currentX = x;
        currentY = y;
        currentHeading = heading;
    }
    lastLeftPos = leftPos;
    lastRightPos = rightPos;
}

int OdometryTracker::liveUpdate() {
    while (true) {
        update(driveBase.getLeftEncoder(), driveBase.getRightEncoder());
        task::sleep(15);
    }
    return 0;
}


void OdometryTracker::update(double leftPos, double rightPos) {

    double dL = driveBase.degToCm(leftPos - lastLeftPos);
    double dR = driveBase.degToCm(rightPos - lastRightPos);

    lastLeftPos = leftPos;
    lastRightPos = rightPos;

    double dS = (dL + dR) / 2.0;

    double dThetaDeg =
        ((dR - dL) / driveBase.baseWidth) * RAD_TO_DEG;

    double headingRad;
    {
        Lock lock(currentHeadingMutex);
        if (trustInert) {
            currentHeading = inert.getTrueHeading();
        }
        headingRad = currentHeading * DEG_TO_RAD;
    }
    double dThetaRad  = dThetaDeg * DEG_TO_RAD;

    if (fabs(dThetaRad) < 1e-6) {
        {
            Lock lockX(currentXMutex);
            Lock lockY(currentYMutex);
            currentX += dS * sin(headingRad); // 0° = +Y
            currentY += dS * cos(headingRad);
        }
    } else {
        double R = dS / dThetaRad;
        {
            Lock lockX(currentXMutex);
            Lock lockY(currentYMutex);
            currentX += R * (sin(headingRad + dThetaRad) - sin(headingRad));
            currentY += R * (cos(headingRad) - cos(headingRad + dThetaRad));
        }
    }

    {
        Lock lock(currentHeadingMutex);
        currentHeading += dThetaDeg;

        if (currentHeading >= 360) {
            currentHeading -= 360;
        }
        if (currentHeading < 0) {
            currentHeading += 360;
        }
    }
}


double OdometryTracker::getX() { 
    Lock lockX(currentXMutex);
    return currentX; 
}
double OdometryTracker::getY() { 
    Lock lockY(currentYMutex);
    return currentY; 
}
double OdometryTracker::getHeading() { 
    Lock lock(currentHeadingMutex);
    return currentHeading; 
}

double OdometryTracker::distanceTo(double targetX, double targetY) {
    double x, y, h;
    getPosition(x, y, h);  // Atomic read
    double dx = targetX - x;
    double dy = targetY - y;
    return sqrt(dx*dx + dy*dy);
}

double OdometryTracker::angleTo(double targetX, double targetY) {
    double x, y, h;
    getPosition(x, y, h);  // Atomic read
    double dx = targetX - x;
    double dy = targetY - y;
    
    double angle = atan2(dx, dy) * RAD_TO_DEG;  // 0° = +Y
    double turnAngle = angle - h;
    
    // Normalize to [-180, 180]
    if (turnAngle > 180) turnAngle -= 360;
    if (turnAngle < -180) turnAngle += 360;
    return turnAngle;
}

void OdometryTracker::getPosition(double& x, double& y, double& heading) {
    Lock lockX(currentXMutex);
    Lock lockY(currentYMutex);
    Lock lockH(currentHeadingMutex);
    x = currentX;
    y = currentY;
    heading = currentHeading;
}

void OdometryTracker::getDistanceAndAngle(double targetX, double targetY, 
                                          double& distance, double& angle) {
    double x, y, h;
    getPosition(x, y, h);  // Atomic read
    
    double dx = targetX - x;
    double dy = targetY - y;
    
    distance = sqrt(dx*dx + dy*dy);
    
    double targetAngle = atan2(dx, dy) * RAD_TO_DEG;
    angle = targetAngle - h;
    
    // Normalize to [-180, 180]
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
}

void OdometryTracker::setTrustInert(bool trust) {
    trustInert = trust;
}