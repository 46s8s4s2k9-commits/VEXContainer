// ControlUnit.cpp - Updated with separate accel/decel limits

#include "ControlUnit.hpp"
#include "vex.h"
#include <cmath>

using namespace vex;

/**
 * @brief Constructor - initializes with separate accel/decel defaults
 * 
 * Default acceleration limits:
 * - Straight accel: 800%/s (moderate limiting for anti-tip)
 * - Straight decel: 1200%/s (can brake harder than accelerate)
 * - Turn accel: 2000%/s (less limiting for responsiveness)
 * - Turn decel: 2500%/s (can stop turns quickly)
 */
ControlUnit::ControlUnit(RobotState& state, SixWheelDrive& driveBase) 
    : state(state), driveBase(driveBase), lastLSpd(0), lastRSpd(0), 
      lastTime(0), maxAccelStraight(800.0), maxDecelStraight(1200.0),
      maxAccelTurn(2000.0), maxDecelTurn(2500.0), 
      transitionIndex(1.0), updateRate(20) {
    lastTime = Brain.Timer.time(msec) + state.trueTimeCorr;
}

/**
 * @brief Applies acceleration or deceleration limiting with intelligent detection
 * 
 * This function determines whether a speed change represents acceleration or
 * deceleration by analyzing the relationship between current and target speeds:
 * 
 * ACCELERATION (speeding up):
 * - Magnitude is increasing: |target| > |last|
 * - Examples: 0→50, 30→60, -30→-60, -20→0→20
 * 
 * DECELERATION (slowing down):
 * - Magnitude is decreasing: |target| < |last|
 * - Examples: 50→0, 60→30, -60→-30, 80→-10
 * 
 * EDGE CASES HANDLED:
 * 1. Direction change through zero: Uses appropriate limit for each phase
 * 2. Sign change without zero: Treated as deceleration (safer)
 * 3. Zero target: Always deceleration
 * 4. Zero last: Always acceleration
 * 
 * @param targetSpd Desired speed (-100 to 100)
 * @param lastSpd Previous speed (-100 to 100)
 * @param deltaTime Time since last update (seconds)
 * @param maxAccel Max acceleration limit (percent/sec)
 * @param maxDecel Max deceleration limit (percent/sec)
 * @return Speed limited to respect accel/decel constraints
 */
double ControlUnit::applyAccelDecelLimit(double targetSpd, double lastSpd, double deltaTime, 
                                         double maxAccel, double maxDecel) {
    // Prevent division by zero or negative time
    if (deltaTime <= 0) {
        return lastSpd;
    }
    
    // Prevent unreasonably large time steps (likely from initialization)
    if (deltaTime > 0.1) {  // 100ms max
        Lock lock(updateRateMutex);
        deltaTime = updateRate / 1000.0;  // Default to 20ms
    }
    
    // Calculate the change in speed
    double deltaSpd = targetSpd - lastSpd;
    
    // If no change needed, return target
    if (fabs(deltaSpd) < 0.01) {
        return targetSpd;
    }
    
    // Determine if this is acceleration or deceleration
    // Key insight: Acceleration increases magnitude, deceleration decreases it
    bool isAccelerating;
    
    if (fabs(lastSpd) < 0.01) {
        // Starting from stop - always acceleration
        isAccelerating = true;
    }
    else if (fabs(targetSpd) < 0.01) {
        // Coming to stop - always deceleration
        isAccelerating = false;
    }
    else if ((lastSpd > 0 && targetSpd > 0) || (lastSpd < 0 && targetSpd < 0)) {
        // Same direction - compare magnitudes
        isAccelerating = fabs(targetSpd) > fabs(lastSpd);
    }
    else {
        // Direction change - this is complex
        // We're crossing through zero or making a sharp reversal
        // Treat as deceleration (safer) unless we're very close to zero
        if (fabs(lastSpd) < 5.0) {
            // Close to zero, treat new direction as acceleration
            isAccelerating = true;
        } else {
            // Far from zero, must decelerate first
            isAccelerating = false;
        }
    }
    
    // Select the appropriate limit
    double maxChange = (isAccelerating ? maxAccel : maxDecel) * deltaTime;
    
    // Apply the limit
    if (fabs(deltaSpd) > maxChange) {
        // Change exceeds limit - clamp it
        double limitedDelta = (deltaSpd > 0) ? maxChange : -maxChange;
        return lastSpd + limitedDelta;
    }
    
    // Within limits - return target
    return targetSpd;
}

/**
 * @brief Calculates turn factor - unchanged from original
 */
double ControlUnit::calculateTurnFactor(double lSpd, double rSpd) {
    double avgSpd = (lSpd + rSpd) / 2.0;
    double turnComponent = fabs(lSpd - rSpd);
    
    if (fabs(lSpd) < 5 && fabs(rSpd) < 5) {
        return 0.0;
    }
    
    double maxSpd = fmax(fabs(lSpd), fabs(rSpd));
    
    if (maxSpd < 1.0) {
        return 0.0;
    }
    
    double turnRatio = turnComponent / (2.0 * maxSpd);
    double turnFactor = pow(turnRatio, transitionIndex);
    
    if (turnFactor > 1.0) turnFactor = 1.0;
    if (turnFactor < 0.0) turnFactor = 0.0;
    
    return turnFactor;
}

/**
 * @brief Main drive function with separate accel/decel limiting
 * 
 * This function applies intelligent limiting to each motor side independently:
 * 1. Calculate time elapsed
 * 2. Clamp input speeds to valid range
 * 3. Calculate turn factor
 * 4. Blend accel/decel limits based on turn factor
 * 5. Apply limits to each side independently
 * 6. Send to drivetrain
 * 
 * Each side gets its own accel/decel determination, so one side can be
 * accelerating while the other decelerates during direction changes.
 */
void ControlUnit::controlledDrive(double lSpd, double rSpd, bool fromAuton) {
    // Get current time
    int32_t currentTime = Brain.Timer.time(msec) + state.trueTimeCorr;
    
    // Calculate time elapsed in seconds
    double deltaTime = (currentTime - lastTime) / 1000.0;

    // Clamp input speeds to valid range
    if (lSpd > 100) lSpd = 100;
    else if (lSpd < -100) lSpd = -100;

    if (rSpd > 100) rSpd = 100;
    else if (rSpd < -100) rSpd = -100;

    double linearSpd = (lSpd + rSpd)/2;

    double turnSpd = fabs(lSpd - linearSpd);

    double turnR = driveBase.baseWidth/(turnSpd/linearSpd);

    double lim = deltaTime;

    if (fabs(linearSpd - lastLinearSpd)>lim) {

    }

    double limitedLSpd;
    double limitedRSpd;
    
    
    // Send to drivetrain
    if (fromAuton) {
        driveBase.lrDrive(limitedLSpd, limitedRSpd);
    } else {
        driveBase.basic_drive(limitedLSpd, limitedRSpd);
    }
    
    // Update tracking variables
    lastTime = currentTime;
    lastLinearSpd = linearSpd;
}



// Setters
void ControlUnit::setMaxAccelerationStraight(double accel) {
    if (accel > 0) {
        maxAccelStraight = accel;
    }
}

void ControlUnit::setMaxDecelerationStraight(double decel) {
    if (decel > 0) {
        maxDecelStraight = decel;
    }
}

void ControlUnit::setMaxAccelerationTurn(double accel) {
    if (accel > 0) {
        maxAccelTurn = accel;
    }
}

void ControlUnit::setMaxDecelerationTurn(double decel) {
    if (decel > 0) {
        maxDecelTurn = decel;
    }
}

void ControlUnit::setTransitionIndex(double index) {
    if (index < 0.5) {
        index = 0.5;
    }
    if (index > 10.0) {
        index = 10.0;
    }
    transitionIndex = index;
}

// Getters
double ControlUnit::getMaxAccelerationStraight() {
    return maxAccelStraight;
}

double ControlUnit::getMaxDecelerationStraight() {
    return maxDecelStraight;
}

double ControlUnit::getMaxAccelerationTurn() {
    return maxAccelTurn;
}

double ControlUnit::getMaxDecelerationTurn() {
    return maxDecelTurn;
}

double ControlUnit::getTransitionIndex() {
    return transitionIndex;
}

void ControlUnit::reset() {
    lastLSpd = 0;
    lastRSpd = 0;
    lastTime = Brain.Timer.time(msec) + state.trueTimeCorr;
}

// Joystick curve functions - unchanged
double ControlUnit::curvedFwdJoystick(double fwdValue) {
    double sign = (fwdValue >= 0) ? 1.0 : -1.0;
    return sign * pow(fabs(fwdValue / 100), fwdCurve) * 100;
}

double ControlUnit::curvedRghtJoystick(double rghtValue) {
    double sign = (rghtValue >= 0) ? 1.0 : -1.0;
    return sign * pow(fabs(rghtValue / 100), rghtCurve) * 100;
}

void ControlUnit::setCurrUpdateRate(double rate) {
    Lock lock(updateRateMutex);
    updateRate = rate;
}

void ControlUnit::setFwdCurve(double index) {
    fwdCurve = index;
}

void ControlUnit::setRghtCurve(double index) {
    rghtCurve = index;
}