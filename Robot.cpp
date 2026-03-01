// Robot.cpp

#include "Robot.hpp"

using namespace vex;

/**
 * @brief Robot class constructor - initializes all subsystems in dependency order
 * 
 * CRITICAL: The initialization order matters! Dependencies must be initialized
 * before the subsystems that use them:
 * 
 * 1. Hardware references (motors, sensors) - stored as references
 * 2. RobotState - needs IMU, no other dependencies
 * 3. SixWheelDrive - needs drive motors, no other dependencies
 * 4. SBallPath - needs intake/outtake/end motors and state
 * 5. OdometryTracker - needs driveBase for encoder access
 * 6. Auton - needs state, driveBase, and odometry
 * 7. ControlUnit - needs state and driveBase
 * 8. PneumaticsTracker - no dependencies
 * 9. ScoringMech - needs intake/outtake/end motors
 * 10. AutonCommands - needs auton, odometry, state, scoringMech
 * 11. DataLogger - no dependencies
 * 12. BallTracker - needs optIn, optOut, state, scoringMech, ballPath
 * 
 * @param lf Left front drive motor
 * @param lm Left middle drive motor
 * @param lb Left back drive motor
 * @param rf Right front drive motor
 * @param rm Right middle drive motor
 * @param rb Right back drive motor
 * @param intake Intake motor
 * @param outtake Outtake motor
 * @param end End effector motor
 * @param optIn Intake optical sensor
 * @param optOut Outtake optical sensor
 * @param imu Inertial measurement unit
 */
Robot::Robot(motor& lf, motor& lm, motor& lb, 
             motor& rf, motor& rm, motor& rb, 
             motor& intake, motor& outtake, motor& end, 
             optical& optIn, optical& optOut, inertial& imu)
    // Initialize hardware references first
    : lf(lf), lm(lm), lb(lb), 
      rf(rf), rm(rm), rb(rb), 
      intake(intake), outtake(outtake), end(end),
      optIn(optIn), optOut(optOut), imu(imu),
      // Initialize core subsystems in dependency order
      inert(imu),                                    // State needs IMU only
      driveBase(lf, lm, lb, rf, rm, rb),           // DriveBase needs motors
      ballPath(intake, outtake, end, state),       // BallPath needs motors and state
      odometry(driveBase, inert),    
      state(inert, odometry),                      
      control(state, driveBase),               // ControlUnit needs state and driveBase
      auton(state, driveBase, odometry, control),           // Auton needs state, driveBase, odometry
      // Initialize auxiliary subsystems
      pnTracker(),                                  // PneumaticsTracker has no dependencies
      // scoringMech(intake, outtake, end),           // ScoringMech needs motors
      autonCommands(auton, odometry, state, ballPath), // AutonCommands needs multiple subsystems
      sdCard(),                                     // DataLogger has no dependencies
      chain(optIn, optOut, state, ballPath) // BallTracker needs sensors and subsystems
{
    // Constructor body is empty - all initialization happens in initializer list
    // This ensures proper construction order and exception safety
}