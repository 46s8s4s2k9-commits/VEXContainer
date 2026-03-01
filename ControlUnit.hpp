// ControlUnit.hpp - Updated with separate accel/decel limits

#ifndef CONTROLUNIT_HPP
#define CONTROLUNIT_HPP

#include "vex.h"
#include "SixWheelDrive.hpp"
#include "RobotState.hpp"
#include "Lock.hpp"

extern vex::brain Brain;

/**
 * @class ControlUnit
 * @brief Provides controlled motor output with separate acceleration and deceleration limiting
 * 
 * This class wraps the SixWheelDrive basic_drive function with intelligent acceleration
 * and deceleration limiting. It uses separate limits for speeding up vs slowing down to
 * account for center of mass imbalances and prevent tipping.
 */
class ControlUnit {
private:
    RobotState& state;
    SixWheelDrive& driveBase;

    double updateRate;
    vex::mutex updateRateMutex;

    double fwdCurve;
    double rghtCurve;
    
    // Last commanded speeds in percent (-100 to 100)
    double lastLSpd;
    double lastRSpd;
    double lastLinearSpd;
    
    // Last update time in milliseconds
    double lastTime;
    
    // Maximum allowed acceleration/deceleration in percent per second
    double maxAccelStraight;     // Speeding up while going straight
    double maxDecelStraight;     // Slowing down while going straight
    double maxAccelTurn;         // Speeding up while turning
    double maxDecelTurn;         // Slowing down while turning
    
    // Transition sharpness control
    double transitionIndex;
    
    /**
     * @brief Applies acceleration or deceleration limiting to a single side's speed
     * 
     * Intelligently determines whether the speed change is acceleration or deceleration
     * and applies the appropriate limit. Handles all edge cases including:
     * - Direction changes (positive to negative or vice versa)
     * - Zero crossings
     * - Magnitude increases vs decreases
     * 
     * @param targetSpd Target speed in percent (-100 to 100)
     * @param lastSpd Last commanded speed in percent (-100 to 100)
     * @param deltaTime Time elapsed since last update in seconds
     * @param maxAccel Maximum allowed acceleration (percent/sec)
     * @param maxDecel Maximum allowed deceleration (percent/sec)
     * @return Limited speed that respects accel/decel constraints
     */
    double applyAccelDecelLimit(double targetSpd, double lastSpd, double deltaTime, 
                                double maxAccel, double maxDecel);
    
    /**
     * @brief Calculates a turn factor (0.0 = straight, 1.0 = full turn)
     */
    double calculateTurnFactor(double lSpd, double rSpd);
    
public:
    ControlUnit(RobotState& state, SixWheelDrive& driveBase);
    
    void controlledDrive(double lSpd, double rSpd, bool fromAuton = false);
    
    // Setters for acceleration limits
    void setMaxAccelerationStraight(double accel);
    void setMaxDecelerationStraight(double decel);
    void setMaxAccelerationTurn(double accel);
    void setMaxDecelerationTurn(double decel);
    void setTransitionIndex(double index);
    
    // Getters
    double getMaxAccelerationStraight();
    double getMaxDecelerationStraight();
    double getMaxAccelerationTurn();
    double getMaxDecelerationTurn();
    double getTransitionIndex();
    
    void reset();
    double curvedFwdJoystick(double fwdValue);
    double curvedRghtJoystick(double rghtValue);
    void setFwdCurve(double index);
    void setRghtCurve(double index);
    void setCurrUpdateRate(double rate);
};

#endif // CONTROLUNIT_HPP