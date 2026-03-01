// OdometryTracker.hpp

#ifndef ODOMETRYTRACKER_HPP
#define ODOMETRYTRACKER_HPP

#include "SixWheelDrive.hpp"
#include "RobotState.hpp"
#include "vex.h"

extern vex::brain Brain;

class OdometryTracker {
public:
    OdometryTracker(SixWheelDrive& driveBase, IMU& inert);
    void reset(double x, double y, double heading, double leftPos, double rightPos);
    int liveUpdate();
    void update(double leftPos, double rightPos);
    double getX();
    double getY();
    double getHeading();
    double distanceTo(double targetX, double targetY);
    double angleTo(double targetX, double targetY);
    void getPosition(double& x, double& y, double& heading);
    void getDistanceAndAngle(double targetX, double targetY, double& distance, double& angle);
    void setTrustInert(bool trust);

private:
    bool trustInert;
    IMU& inert;
    double currentX;
    vex::mutex currentXMutex;
    double currentY;
    vex::mutex currentYMutex;
    double currentHeading; // in degrees
    vex::mutex currentHeadingMutex;
    double lastLeftPos;
    double lastRightPos;
    SixWheelDrive& driveBase;
    static constexpr double DEG_TO_RAD = M_PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / M_PI;
};

#endif // ODOMETRYTRACKER_HPP
