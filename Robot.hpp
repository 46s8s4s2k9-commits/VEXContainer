// Robot.hpp

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "vex.h"
#include "RobotState.hpp"
#include "SixWheelDrive.hpp"
#include "Auton.hpp"
#include "OdometryTracker.hpp"
#include "PneumaticsTracker.hpp"
#include "AutonCommands.hpp"
#include "DataLogger.hpp"
#include "BallTracker.hpp"
#include "SBallPath.hpp"
#include "ControlUnit.hpp"

extern vex::brain Brain;

/**
 * @class Robot
 * @brief Central robot class that integrates all subsystems
 * 
 * This class serves as the main container for all robot subsystems including:
 * - Drive base (motors and control)
 * - Scoring mechanisms (intake, outtake, end effector)
 * - Autonomous control (PID, odometry)
 * - Ball tracking system
 * - Data logging
 * - Pneumatics tracking
 * 
 * The Robot class handles initialization and provides centralized access to
 * all robot components through a single object instance.
 */
class Robot {
public:
    // Hardware references - Motors
    vex::motor& lf;      // Left front drive motor
    vex::motor& lm;      // Left middle drive motor
    vex::motor& lb;      // Left back drive motor
    vex::motor& rf;      // Right front drive motor
    vex::motor& rm;      // Right middle drive motor
    vex::motor& rb;      // Right back drive motor
    vex::motor& intake;  // Intake motor
    vex::motor& outtake; // Outtake motor
    vex::motor& end;     // End effector motor
    
    // Hardware references - Sensors
    vex::inertial& imu;  // Inertial measurement unit
    vex::optical& optIn; // Optical sensor at intake
    vex::optical& optOut; // Optical sensor at outtake

    // Core subsystems (order matters for initialization)
    IMU inert;
    RobotState state;           // Robot state tracking (heading, position, etc.)
    SixWheelDrive driveBase;    // Drive base motor control
    SBallPath ballPath;         // Ball path control (intake/outtake coordination)
    OdometryTracker odometry;   // Position tracking using wheel encoders
    ControlUnit control;    // Driver control with acceleration limiting
    Auton auton;                // Autonomous PID control
    
    // Auxiliary subsystems
    PneumaticsTracker pnTracker;   // Pneumatics usage tracking
    // ScoringMech scoringMech;       // Scoring mechanism state management
    AutonCommands autonCommands;   // High-level autonomous commands
    DataLogger sdCard;             // SD card data logging
    BallTracker chain;             // Ball tracking system

    /**
     * @brief Constructs the Robot object and initializes all subsystems
     * 
     * @param lf Left front motor reference
     * @param lm Left middle motor reference
     * @param lb Left back motor reference
     * @param rf Right front motor reference
     * @param rm Right middle motor reference
     * @param rb Right back motor reference
     * @param intake Intake motor reference
     * @param outtake Outtake motor reference
     * @param end End effector motor reference
     * @param optIn Intake optical sensor reference
     * @param optOut Outtake optical sensor reference
     * @param imu Inertial measurement unit reference
     * 
     * All subsystems are initialized in the correct dependency order.
     * State must be initialized before any subsystem that depends on it.
     * DriveBase must be initialized before odometry and auton.
     */
    Robot(vex::motor& lf, vex::motor& lm, vex::motor& lb, 
          vex::motor& rf, vex::motor& rm, vex::motor& rb, 
          vex::motor& intake, vex::motor& outtake, vex::motor& end, 
          vex::optical& optIn, vex::optical& optOut, vex::inertial& imu);
};

#endif // ROBOT_HPP