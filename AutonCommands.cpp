// AutonCommands.cpp

#include "AutonCommands.hpp"

using namespace vex;

extern vex::inertial Inertial1;

/**
 * Constructor for AutonCommands.
 * 
 * @param a Auton object, used for pid control
 * @param o OdometryTracker object, used for tracking robot position and heading
 * @param state RobotState object, used for storing important state variables
 * @param in Motor object, used for intake
 * @param out Motor object, used for outtake
 * @param end Motor object, used for end motor
 */
AutonCommands::AutonCommands(Auton& a, OdometryTracker& o, RobotState& state, SBallPath& ballPath) 
    : auton(a), odometry(o), state(state), ballPath(ballPath) {}

/**
 * @brief Move the robot to a target position (x, y) and heading with optional PID control.
 * 
 * @param targetX Target x coordinate in centimeters.
 * @param targetY Target y coordinate in centimeters.
 * @param precise Optional parameter. If true, uses PID control for precise distance and angle movements. If false, uses basic motor spinFor commands for simple movements.
 */
void AutonCommands::moveTo(double targetX, double targetY, bool precise, double finalHeading) {
    double distance, angle;
    
    // Single atomic read of position + calculations
    odometry.getDistanceAndAngle(targetX, targetY, distance, angle);
    
    // Turn to face target
    if (fabs(angle) > turnToleranceDeg) {
        if (angle < 0) {
            auton.PIDturnLeft(fabs(angle));
        } else {
            auton.PIDturnRight(fabs(angle));
        }
        wait(100, msec);
    }
    
    // Re-read after turn (position may have shifted slightly)
    odometry.getDistanceAndAngle(targetX, targetY, distance, angle);
    
    // Drive to target
    auton.PIDdriveForward(distance);
    wait(50, msec);
    

    if (finalHeading != -999) {
        turnToHeading(finalHeading, precise);
    }
}

void AutonCommands::intakeBalls(int durationMs) {
    ballPath.storeBalls();
    wait(durationMs, msec);
    ballPath.stop();
}

/**
 * @brief Scores a high goal by spinning the intake and outtake motors forward and the end motor forward for a specified duration.
 * 
 * @param durationMs The duration in milliseconds to spin the motors for.
 */
void AutonCommands::scoreHighGoal(int durationMs) {
    ballPath.highOut();
    task::sleep(durationMs);
    ballPath.stop();
}

/**
 * @brief Scores a mid goal by spinning the intake and outtake motors forward and the end motor backward for a specified duration.
 * 
 * @param durationMs The duration in milliseconds to spin the motors for.
 */
void AutonCommands::scoreMidGoal(int durationMs) {
    ballPath.midOut();
    task::sleep(durationMs);
    ballPath.stop();
}

/**
 * @brief Turns the robot to a target heading with optional PID control.
 * 
 * @param targetHeading The target heading in degrees.
 * @param precise Optional parameter. If true, uses PID control for precise angle movements. If false, uses basic motor spinFor commands for simple turns.
 */
void AutonCommands::turnToHeading(double targetHeading, bool precise) {
    double x, y, currentHeading;
    odometry.getPosition(x, y, currentHeading);  // Atomic read
    
    double turnAngle = targetHeading - currentHeading;

    if (precise) {
        turnAngle = state.getAngleDifference(Inertial1.heading(), targetHeading);
    }
    
    if (fabs(turnAngle) < turnToleranceDeg) return;
    
    if (turnAngle < 0) {
        auton.PIDturnLeft(fabs(turnAngle));
    } else {
        auton.PIDturnRight(fabs(turnAngle));
    }
}

void AutonCommands::singTurnToHeading(double targetHeading, bool isFwd, bool precise) {
    double x, y, currentHeading;
    odometry.getPosition(x, y, currentHeading);
    
    double turnAngle = targetHeading - currentHeading;

    if (precise) {
        turnAngle = state.getAngleDifference(Inertial1.heading(), targetHeading);
    }
    
    if (fabs(turnAngle) < turnToleranceDeg) return;
    
    if (turnAngle < 0) {
        if (isFwd) {
            auton.PIDsingTurnL(fabs(turnAngle));
        } else {
            auton.PIDsingTurnR(fabs(turnAngle));
        }
    } else {
        if (isFwd) {
            auton.PIDsingTurnR(fabs(turnAngle));
        } else {
            auton.PIDsingTurnL(fabs(turnAngle));
        }
    }
}
