#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
motor LF = motor(PORT14, ratio6_1, true);
motor LM = motor(PORT15, ratio6_1, true);
motor LB = motor(PORT12, ratio6_1, false);

motor RF = motor(PORT19, ratio6_1, false);
motor RM = motor(PORT17, ratio6_1, false);
motor RB = motor(PORT18, ratio6_1, true);

controller Controller1 = controller(primary);
optical Opt = optical(PORT16);
motor Intake = motor(PORT5, ratio6_1, true);
motor Outtake = motor(PORT10, ratio18_1, false);
motor End = motor(PORT7, ratio18_1, true);

digital_out PnFDescore = digital_out(Brain.ThreeWirePort.A);
digital_out PnIntake = digital_out(Brain.ThreeWirePort.B);
digital_out PnSDescore = digital_out(Brain.ThreeWirePort.C);

digital_out PnLoader = digital_out(Brain.ThreeWirePort.D);

optical IntakeOpt = optical(PORT11);
inertial Inertial1 = inertial(PORT13);


// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}

// Converts a color to a string
const char* convertColorToString(color col) {
  if (col == color::red) return "red";
  else if (col == color::green) return "green";
  else if (col == color::blue) return "blue";
  else if (col == color::white) return "white";
  else if (col == color::yellow) return "yellow";
  else if (col == color::orange) return "orange";
  else if (col == color::purple) return "purple";
  else if (col == color::cyan) return "cyan";
  else if (col == color::black) return "black";
  else if (col == color::transparent) return "transparent";
  else return "unknown";
}


void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Bryan Tu                                                  */
/*    Created:      Dec 18, 2025                                              */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
#include <string>
#include <vector>


#include "Robot.hpp"
#include "Lock.hpp"
#include "AutonRoutes.hpp"
#include "PreAutonState.hpp"
#include "ControllerScreen.hpp"
#include <functional>
#include <atomic>


// Allows for easier use of the VEX Library
using namespace vex;


enum class AutonCalc {
  PID,
  Template
};

///////////////////////////////////////////////////
// Variable initialization (Do Not Change)
/////////////////////////////////////////////////////////////////////

Robot robot(LF, LM, LB, RF, RM, RB, Intake, Outtake, End, Opt, IntakeOpt, Inertial1);

// Intake mode state
bool isIntake = false;

// Pneumatics states
bool isFDescoreUp = false;
bool isSDescoreUp = false;
bool isLoaderDown = false;
bool isIntakeUp = false;

bool isMatchEnd = false;
bool isStatDisplayed = false;

CurrEvent currEvent = CurrEvent::PreAuton;

bool isLogEnd = false;

bool isSort = true;

thread ballPathThread;
thread liveupdate;

PreAutonState preAutonState;

ControllerScreen controllerScreen(Controller1);

competition Competition;

///////////////////////////////
// Constant variables
/////////////////////////////////////////////////

bool isMatch = true;

bool teamIsBlue = true;

AutonCalc autonCalculation = AutonCalc::PID;

bool usingBtn = false;

bool usingSd = true;

bool enableControl = false;




///////////////////////////////////////////////////
// Autonomous Route Definitions
/////////////////////////////////////////////////////////////////////

std::vector<AutonRoute> autonRoutes = getAutonRoutes();

// Currently selected route index (default to first route)
int selectedRouteIndex = 1; // Default to "Right"

///////////////////////////////////////////////////
// Helper Functions for Route Management
/////////////////////////////////////////////////////////////////////

/**
 * @brief Get the next route index (wraps around)
 */
int getNextRouteIndex(int current) {
    return (current + 1) % autonRoutes.size();
}

/**
 * @brief Get the previous route index (wraps around)
 */
int getPrevRouteIndex(int current) {
    return (current - 1 + autonRoutes.size()) % autonRoutes.size();
}

/**
 * @brief Get the currently selected route
 */
AutonRoute& getSelectedRoute() {
    return autonRoutes[selectedRouteIndex];
}

// Screen is 479 x 239
void initialization() {
  Opt.integrationTime(50);

  //=============================================================================
  // DRIVE BASE CONFIGURATION
  //=============================================================================
  
  // Drive speed levels (multipliers for controller input)
  robot.driveBase.driveSpdThree = 1.0;   // Level 3: 100% speed (full power)
  robot.driveBase.driveSpdTwo = 0.75;    // Level 2: 75% speed (default)
  robot.driveBase.driveSpdOne = 0.5;     // Level 1: 50% speed (precision)
  robot.driveBase.setDriveSpeed(2);      // Start at speed level 2
  
  // Joystick deadzone (prevents drift from controller stick noise)
  robot.driveBase.joystickDeadzone = 10;  // 10% deadzone
  
  // Motor correction factors (compensate for mechanical differences)
  // Values > 1.0 increase that side's power, < 1.0 decrease it
  // Tune these if robot drifts to one side when driving straight
  robot.driveBase.setCorrections(1.0, 1.0); // Left: 100%, Right: 102%
  
  // Brake system timing
  robot.driveBase.breakMSec = 500; // Time to apply brakes (ms)
  
  // Set motor brake mode
  LF.setStopping(coast); 
  LM.setStopping(coast);
  LB.setStopping(coast);
  RF.setStopping(coast);
  RM.setStopping(coast);
  RB.setStopping(coast);
  
  //=============================================================================
  // ROBOT SPECIFICATIONS (MEASURE THESE CAREFULLY!)
  //=============================================================================
  
  // Drivetrain geometry
  robot.driveBase.baseWidth = 29.0;        // Distance between left/right wheels (cm) (34.3)
  robot.driveBase.wheelDiameter = 8.255;   // Drive wheel diameter (cm)
  robot.driveBase.gearRatio = 48.0/78.0;   // Drive gear ratio (42T motor : 78T wheel) (36/48)
  robot.driveBase.wheelCirc = robot.driveBase.wheelDiameter * M_PI; // Wheel circumference (cm)
  
  //=============================================================================
  // AUTONOMOUS PID CONFIGURATION
  //=============================================================================
  

  robot.auton.setPIDConstants(1.8, 0.03, 0.05);
  
  robot.auton.setTurnPIDConstants(2.0, 0.05, 0);
  
  robot.auton.setPIDLimit(60);     
  robot.auton.setPIDTimeout(6000);  //ms
  
  // Autonomous motor corrections (same concept as driver corrections) Static
  robot.auton.setAutonCorrections(1.0, 1.0);

  robot.auton.setPIDDriveFwdCorr(1.5);

  // Set if inertial sensor is trustworthy during autonomous
  robot.auton.setTrustInert(true);

  robot.odometry.setTrustInert(true);

  robot.auton.setUseControl(false); // false should work, use kd to replace accel limit
  
  //=============================================================================
  // DRIVER CONTROL UNIT CONFIGURATION (Anti-tip system)
  //=============================================================================
  
  // Straight driving limits (percent per second)
  robot.control.setMaxAccelerationStraight(800.0);   // Speeding up straight
  robot.control.setMaxDecelerationStraight(1200.0);  // Slowing down straight
  
  // Turning limits (percent per second)
  robot.control.setMaxAccelerationTurn(2000.0);      // Speeding up in turns
  robot.control.setMaxDecelerationTurn(2500.0);      // Slowing down in turns
  
  // Transition sharpness (1.0 = gradual, 3.0 = sharp)
  robot.control.setTransitionIndex(0.5);
  
  // Joystick curves
  robot.control.setFwdCurve(1);
  robot.control.setRghtCurve(1);
  
  //=============================================================================
  // BALL TRACKING AND OPTICAL SENSOR CONFIGURATION
  //=============================================================================
  
  // Ball detection timing
  robot.chain.setIntakeContBallMSec(200);    // Time to detect continuous ball (ms)
  robot.chain.setIntakeDebounceTime(0);      // Debounce delay for intake sensor (ms)
  robot.chain.setOuttakeContBallMSec(200);   // Time to detect outtake ball (ms)
  robot.chain.setOuttakeDebounceTime(0);     // Debounce delay for outtake sensor (ms)
  
  // Optical sensor color detection ranges (hue values 0-360)
  // Red wraps around 0/360, blue is around 180-260
  // Tune these based on actual ball colors and lighting conditions
  robot.chain.setOptRed(330, 20);    // Red: 330-360 and 0-20 (wraps around)
  robot.chain.setOptBlue(150, 260);  // Blue: 150-260
  
  // Turn on optical sensors
  Opt.setLightPower(100, pct);
  Opt.setLight(ledState::on);
  IntakeOpt.setLightPower(100, pct);
  IntakeOpt.setLight(ledState::on);
  
  //=============================================================================
  // SCORING MECHANISM CONFIGURATION
  //=============================================================================
  
  // End roller speed during intake
  robot.ballPath.setIntakeEndSpd(0);  
  
  //=============================================================================
  // PNEUMATICS TRACKING
  //=============================================================================
  
  // Set maximum pneumatic actuations based on tank size
  // Monitor this during matches to avoid running out of air
  robot.pnTracker.setMax(36);  // Estimated max uses (tune based on tank size)
  robot.pnTracker.reset();     // Reset counter to 0

  controllerScreen.setPn(robot.pnTracker.remaining());
  
  //=============================================================================
  // INITIALIZATION COMPLETE
  //=============================================================================
  
  // Display confirmation message
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Robot Ready");
}

//////////////////////////
// Main Code Loop
////////////////////////////////////////

// Function to control intake and outtake
void intake_outtake() {
  while (true) {
    if (Controller1.ButtonR1.pressing()) {
      while (Controller1.ButtonR1.pressing()) {
        robot.ballPath.storeBalls();
      }
      robot.ballPath.stop();
    }

    // Intake motor reverse if L2 press and hold
    if (Controller1.ButtonR2.pressing()) {
      while (Controller1.ButtonR2.pressing()) {
        robot.ballPath.lowOut();
      }
      robot.ballPath.stop();
      PnIntake.set(false);
      isIntakeUp = false;
    }

    // Full outtake high goal if R1 press and hold
    if (Controller1.ButtonL1.pressing()) {
      while (Controller1.ButtonL1.pressing()) {
        robot.ballPath.highOut();
        if (isSort) {
          if (teamIsBlue) {
            if (robot.chain.isRed(Opt.hue())) {
              robot.ballPath.highRev();
              wait(275, msec);
            }
          } else {
            if (robot.chain.isBlue(Opt.hue())) {
              robot.ballPath.highRev();
              wait(275, msec);
            }
          }
        }
      }
      PnFDescore.set(false);
      isFDescoreUp = false;
      robot.ballPath.stop();
    }

    if (Controller1.ButtonLeft.pressing()) {
      isSort = !isSort;
      controllerScreen.refresh();
      wait(300, msec);
    }

    // Full outtake middle goal if R2 press and hold

    if (Controller1.ButtonL2.pressing()) {
      while (Controller1.ButtonL2.pressing()) {
        robot.ballPath.midOut();
      }
      robot.ballPath.stop();
    }

    if (Controller1.ButtonRight.pressing()) {
      if (teamIsBlue) {
        teamIsBlue = false;
      } else {
        teamIsBlue = true;
      }
      controllerScreen.setTeam(teamIsBlue);
      controllerScreen.refresh();
      Controller1.rumble(".");
      wait(300, msec);
    }
    task::sleep(20);
  }
}

void holdRobot() {
  while (true) {
    if (Controller1.ButtonY.pressing()) {
      while (Controller1.ButtonY.pressing()) {
        robot.driveBase.setHold();
      }
      robot.driveBase.setCoast();
    }
    task::sleep(20);
  }
}

void pneumatics_code() {
  if (Controller1.ButtonDown.pressing()) {
    isSDescoreUp = !isSDescoreUp;  // Toggle state
    PnSDescore.set(isSDescoreUp);
    robot.pnTracker.use(0.5);
    wait(300, msec);
  }

  if (Controller1.ButtonB.pressing()) {
    isLoaderDown = !isLoaderDown;  // Toggle state
    PnLoader.set(isLoaderDown);
    robot.pnTracker.use();
    controllerScreen.setPn(robot.pnTracker.remaining());
    controllerScreen.refresh();
    if (robot.pnTracker.remaining() <= 4.0) {
      Controller1.rumble("..");
    }
    wait(300, msec);
  }
}

// Checks if encoder is working for debugging
void debug() {
  while (true) {
    double lft = robot.auton.degToCm(robot.driveBase.getLeftEncoder());
    double rght = robot.auton.degToCm(robot.driveBase.getRightEncoder());
    double diff = lft - rght;

    Brain.Screen.clearScreen();

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Left(cm): %.2f", lft);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Right(cm): %.2f", rght);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Diff(L-R): %.2f", diff);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Pn Left: %f", robot.pnTracker.remaining());
    Brain.Screen.newLine();
    Brain.Screen.print("CurrX: %.2f", robot.odometry.getX());
    Brain.Screen.newLine();
    Brain.Screen.print("CurrY: %.2f", robot.odometry.getY());
    Brain.Screen.newLine();
    Brain.Screen.print("CurrHeading: %.2f", robot.odometry.getHeading());
    Brain.Screen.newLine();
    Brain.Screen.print("InertHeading: %.2f", robot.state.getTrueHeading());

    printf("Heading: %.2f\n", Inertial1.heading());
    printf("Odom: %.2f\n", robot.odometry.getHeading());
    printf("X: %.2f\n", robot.odometry.getX());
    printf("Y: %.2f\n", robot.odometry.getY());

    task::sleep(100);
  }
}

void driveSpdChange() {
  if (Controller1.ButtonUp.pressing()) {
    if (robot.driveBase.driveSpd < 3) {
      robot.driveBase.driveSpd += 1;
      wait(150, msec);
    }
  }
  if (Controller1.ButtonDown.pressing()) {
    if (robot.driveBase.driveSpd > 1) {
      robot.driveBase.driveSpd -= 1;
      wait(150, msec);
    }
  }
}

void refreshScreen(int index) {
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(vex::white);
  
  Brain.Screen.setFont(mono30);
  Brain.Screen.setCursor(1, 1);
  if (index == 1) {
    if (isMatch) {
      Brain.Screen.print("*Mode: Competition");
    } else {
      Brain.Screen.print("*Mode: Test");
    }
  } else {
    if (isMatch) {
      Brain.Screen.print("Mode: Competition");
    } else {
      Brain.Screen.print("Mode: Test");
    }
  }

  Brain.Screen.setCursor(2,1);
  if (index == 2) {
    if (preAutonState.duringCal) {
      Brain.Screen.setPenColor(color::blue);
      Brain.Screen.print("*Inertial: Calibrating...");
      Brain.Screen.setPenColor(color::white);
    } else if (preAutonState.inertCal) {
      if (!preAutonState.isAtRest) {
        Brain.Screen.setPenColor(color::yellow);
      }
      Brain.Screen.print("*Inertial: Calibrated √");
      Brain.Screen.setPenColor(color::white);
    } else {
      Brain.Screen.setPenColor(color::red);
      Brain.Screen.print("*Inertial: Calibrate");
      Brain.Screen.setPenColor(color::white);
    }
  } else {
    if (preAutonState.duringCal) {
      Brain.Screen.setPenColor(color::blue);
      Brain.Screen.print("Inertial: Calibrating...");
      Brain.Screen.setPenColor(color::white);
    } else if (preAutonState.inertCal) {
      if (!preAutonState.isAtRest) {
        Brain.Screen.setPenColor(color::yellow);
      }
      Brain.Screen.print("Inertial: Calibrated √");
      Brain.Screen.setPenColor(color::white);
    } else {
      Brain.Screen.setPenColor(color::red);
      Brain.Screen.print("Inertial: Calibrate");
      Brain.Screen.setPenColor(color::white);
    }
  }

  // Display selected mode at top
  Brain.Screen.setFont(mono30);
  Brain.Screen.setCursor(3, 1);
  if (index == 3) {
    Brain.Screen.print("*Selected:");
  } else {
    Brain.Screen.print("Selected:");
  }
  
  Brain.Screen.setFont(mono30);
  Brain.Screen.setCursor(4, 1);
  if (index == 3) {
    Brain.Screen.print("*%s", getSelectedRoute().name.c_str());
  } else {
    Brain.Screen.print("%s", getSelectedRoute().name.c_str());
  }

  Brain.Screen.setFont(mono30);
  Brain.Screen.setCursor(5,1);
  if (index == 4) {
    if (teamIsBlue) {
      Brain.Screen.print("*Blue");
    } else {
      Brain.Screen.print("*Red");
    }
  } else {
    if (teamIsBlue) {
      Brain.Screen.print("Blue");
    } else {
      Brain.Screen.print("Red");
    }
  }

  Brain.Screen.setCursor(5, 8);
  Brain.Screen.print("H: %.2f", preAutonState.heading);
  
  // Display route number
  Brain.Screen.setFont(mono20);
  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("Route %d of %d", selectedRouteIndex + 1, (int)autonRoutes.size());
  
  // Display instructions at bottom
  Brain.Screen.setFont(mono20);
  Brain.Screen.setCursor(12, 1);
  if (!preAutonState.isScreenLocked) {
    Brain.Screen.print("L/R: Cycle | X: Lock | U/D: Switch | A: OK");
  } else {
    Brain.Screen.print("LOCKED - Ready for match");
  }
}

int preAutonomous(void) {
  //=============================================================================
  // INITIALIZATION
  //=============================================================================
  
  // Set current event
  currEvent = CurrEvent::PreAuton;
  
  // Turn on optical sensors for ball detection
  Opt.setLightPower(100, pct);
  Opt.setLight(ledState::on);
  IntakeOpt.setLightPower(100, pct);
  IntakeOpt.setLight(ledState::on);
  int lineIndex = 3;
  const int maxIndex = 4;
  
  // Display initial screen
  refreshScreen(lineIndex);
  
  //=============================================================================
  // SELECTION LOOP (runs until locked)
  //=============================================================================
  
  while (!preAutonState.isScreenLocked) {

    // State Monitoring
    if (preAutonState.currLoop >= preAutonState.updateLoop) {
      if (robot.inert.isAtRest() && !preAutonState.isAtRest) {
        preAutonState.isAtRest = true;
        refreshScreen(lineIndex);
      } else if (!robot.inert.isAtRest() && preAutonState.isAtRest) {
        preAutonState.isAtRest = false;
        refreshScreen(lineIndex);
      }
      preAutonState.heading = robot.inert.getTrueHeading();
      if (fabs(robot.state.getAngleDifference(preAutonState.heading, 0))>3) {
        refreshScreen(lineIndex);
      }
      preAutonState.currLoop = 0;
    } else {
      preAutonState.currLoop += 1;
    }



    //-------------------------------------------------------------------------
    // CONTROLLER SELECTION
    //-------------------------------------------------------------------------
    
    // Right button - cycle forward through routes
    if (Controller1.ButtonRight.pressing()) {
      if (lineIndex == 1) {
        if (isMatch) {
          isMatch = false;
        } else {
          isMatch = true;
        }
      }
      if (lineIndex == 3) {
        selectedRouteIndex = getNextRouteIndex(selectedRouteIndex);
      }
      if (lineIndex == 4) {
        if (teamIsBlue) {
          teamIsBlue = false;
        } else {
          teamIsBlue = true;
        }
      }
      
      refreshScreen(lineIndex);
      
      // Haptic feedback
      Controller1.rumble(".");
      
      wait(200, msec);  // Debounce
    }
    
    // Left button - cycle backward through routes
    if (Controller1.ButtonLeft.pressing()) {
      if (lineIndex == 3) {
        selectedRouteIndex = getPrevRouteIndex(selectedRouteIndex);
      }
      
      refreshScreen(lineIndex);
      
      // Haptic feedback
      Controller1.rumble(".");
      
      wait(200, msec);  // Debounce
    }

    if (Controller1.ButtonUp.pressing()) {
      if (lineIndex != 1) {
        lineIndex -= 1;
      }
      refreshScreen(lineIndex);

      Controller1.rumble(".");

      wait(200, msec);
    }

    if (Controller1.ButtonDown.pressing()) {
      if (lineIndex != maxIndex) {
        lineIndex += 1;
      }
      refreshScreen(lineIndex);

      Controller1.rumble(".");
      wait(200, msec);
    }

    if (Controller1.ButtonA.pressing()) {
      if (lineIndex == 2) {
        if (!robot.inert.isCalibrating()) {
          robot.inert.calibrate();
          preAutonState.duringCal = true;
          refreshScreen(lineIndex);
          while (robot.inert.isCalibrating()) {
            wait(50, msec);
          }
          preAutonState.duringCal = false;
          preAutonState.inertCal = true;
        } else {
          Controller1.rumble("..");
        }
      }

      refreshScreen(lineIndex);
      Controller1.rumble(".");
      wait(200,msec);
    }
    
    //-------------------------------------------------------------------------
    // LOCK SELECTION
    //-------------------------------------------------------------------------
    
    // A or X button - lock the current selection
    if (Controller1.ButtonX.pressing()) {
      preAutonState.isScreenLocked = true;
      
      // Update screen to show locked state
      Brain.Screen.clearScreen();
      Brain.Screen.setFont(mono60);
      Brain.Screen.setCursor(5, 5);
      Brain.Screen.print("LOCKED!");
      
      Brain.Screen.setFont(mono30);
      Brain.Screen.setCursor(7, 3);
      Brain.Screen.print("Route: %s", getSelectedRoute().name.c_str());
      
      // Start data logging if enabled
      if (usingSd) {
        robot.sdCard.startLogging();
        Brain.Screen.setFont(mono20);
        Brain.Screen.setCursor(10, 1);
        Brain.Screen.print("Logging: %s", robot.sdCard.getFilename().c_str());
      }

      if (!preAutonState.inertCal && !robot.inert.isCalibrating()) {
        Controller1.rumble("..");
        robot.inert.calibrate();
        while (robot.inert.isCalibrating()) {
          wait(50, msec);
        }
      } else if (robot.inert.isCalibrating()) {
        Controller1.rumble("..");
        while (robot.inert.isCalibrating()) {
          wait(50, msec);
        }
      }
      // Haptic feedback - single pulse
      Controller1.rumble(".");
      
      wait(500, msec);
    }
    
    //-------------------------------------------------------------------------
    // LOOP TIMING
    //-------------------------------------------------------------------------
    
    task::sleep(50);  // 50ms loop rate (20Hz)
  }
  
  //=============================================================================
  // WAITING FOR MATCH START
  //=============================================================================
  
  // If in competition mode, wait here until autonomous starts
  if (isMatch) {
    Brain.Screen.setFont(mono40);
    Brain.Screen.setCursor(9, 4);
    Brain.Screen.print("Ready for match");
    
    // Infinite loop - competition object will break us out
    while (true) {
      wait(50, msec);
    }
  }
  
  return 0;
}

///////////////////////////////////////////////////
// Main Autonomous Function
/////////////////////////////////////////////////////////////////////

int dataLog() {
  while (!robot.sdCard.isLogging) {
    task::sleep(100);
  }
  while (!isLogEnd) {
    robot.sdCard.logData(robot.driveBase, robot.odometry, robot.pnTracker, robot.state, currEvent, Controller1.Axis3.position(), Controller1.Axis1.position(), Opt.hue());
    task::sleep(200);
  }
  robot.sdCard.stopLogging();
  return 0;
}

void autonomous(void) {
  currEvent = CurrEvent::Auton;
  printf("Autonomous period\n");
  
  // Reset timer at start of autonomous
  robot.state.trueTimeCorr += Brain.Timer.time(msec);
  Brain.Timer.reset();
  robot.pnTracker.reset();
  robot.driveBase.resetEnc(0, 0);
  robot.control.reset();
  printf("1\n");

  robot.driveBase.setBrake();
  
  if (!preAutonState.inertCal || robot.inert.isCalibrating()) {
    robot.auton.setTrustInert(false);
    robot.odometry.setTrustInert(false);
  }
  robot.inert.resetAll();
  printf("2\n");

  {
    Lock lock(pathCmdMutex);
    pathCmd = ballPathCommand::None;
  }

  ballPathThread = thread(ballPathLoop);
  printf("3\n");
  
  // Execute the selected autonomous route
  getSelectedRoute().execute();
  printf("4\n");
  
  return;
}

void userControl(void) {
  currEvent = CurrEvent::Driver;
  printf("Driver Control\n");

  liveupdate.interrupt();

  {
    Lock lock(pathCmdMutex);
    pathCmd = ballPathCommand::None;
  }
  ballPathThread.interrupt();

  controllerScreen.setTeam(teamIsBlue);
  controllerScreen.refresh();
  
  // Reset timer at start of driver control
  robot.state.trueTimeCorr += Brain.Timer.time(msec);
  Brain.Timer.reset();
  robot.control.reset();

  robot.ballPath.stop();

  robot.driveBase.setCoast();

  thread ballPathControl = thread(intake_outtake);
  thread monitorHoldBtn = thread(holdRobot);
  
  while (true) {
    double ax3 = robot.control.curvedFwdJoystick(Controller1.Axis3.position());
    double ax1 = robot.control.curvedRghtJoystick(Controller1.Axis1.position());
    double lft_spd = (ax3 + ax1);
    double rght_spd = (ax3 - ax1);

    if (enableControl) {
      robot.control.controlledDrive(lft_spd, rght_spd);
    } else {
      robot.driveBase.basic_drive(lft_spd, rght_spd);
    }

    // Pneumatics code
    pneumatics_code();

    // Drive speed change
    driveSpdChange();

    if (usingSd) {
      if (!Competition.isEnabled() && currEvent == CurrEvent::Driver) {
        isMatchEnd = true;
        if (!isStatDisplayed) {
          isStatDisplayed = true;
          robot.sdCard.displayStats();
        }
      }
    }

    wait(20, msec);
  }
}

void odometryTaskWrapper() {
  robot.odometry.liveUpdate();
}


void skillsTimer() {
  
}

int main() {
  // Initializing Robot Configuration
  vexcodeInit();
  
  // Reset timer at start
  Brain.Timer.reset();
  initialization();

  printf("Start Program\n");

  liveupdate = thread(odometryTaskWrapper);
  liveupdate.setPriority(thread::threadPriorityHigh);

  robot.control.reset();


  if (usingSd) {
    task datalog(dataLog);
  }

  /*
  bool start = false;
  while (!start) {
    if (Controller1.ButtonA.pressing()) {
      start = true;
      wait(300, msec);
    }
    wait(20, msec);
  }
  int startTime = Brain.Timer.time();*/
  //userControl();

  /*
  if (isMatch) {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(userControl);
  }*/

  // Pre-autonomous selection
  preAutonomous();
  printf("%.2f\n", Inertial1.heading());
  //autonomous();

  thread debugThread;
  if (!isMatch) {
    debugThread = thread(debug);
    autonomous();
    userControl();
  }

  while (true) {
    wait(100, msec);
  }
}


// TODO: change basewidth var, measure by turning and encoder values
// use zerotimed used var in sixwheel class âˆš
// Add controller joystick deadzone âˆš
// Add change motor break to hold if the bot drifts too much after controller input is near 0 for 0.5 seconds âˆš
// Easy auton selection, both controller selction and brain touch screen selection, use during preauton selection âˆš
// Reset timer every event (preauton, auton, driver) âˆš
// Add none customized PID if no time to tune âˆš
// Pneumatics estimation algorithm, Add after main code done (how many pneumatics can I use left) âˆš
// TODO: Auton Yaw correction If motor power inconsistancy, correct, use odometry functions to live calculate and store current coordinate. Still needs testing
// TODO: Auton easy creation algorithm, simple commands like outtake balls at left long goal, intake balls at a coordinate, matchload, etc, calculates best path, with variables like prioritizing accuracy or speed etc. Smart avoid preset obstacles (leave space or endpoints to add sensor tracking and obstacle avoiding later.)
// TODO: Perfect arc pid drive
// TOD Auton Speed anti tipping √
// TOD Driver anti tipping √
// TOD Driver controller curve √
// TODO: Measure robot specs, center of turn etc.
// TODO: Driver assist programs
// TODO: Double controller if needed (for driver assist programs control)
// TODO: Positioning system: Inertial reference and heading ref
// TODO: Positioning system: Distance sensor and heading
// TODO: Positioning system: Odometry tune and change basewidth
// TODO: Positioning system: AI Vision Sensor game object detection / static game object reference positioning system.
// TOD "BallTracker.cpp" : Finish update(), add roller monitoring class first √
// TODO: Auton time estimation using speed aproximation and integration, using a wheel slip/efficiency loss index from testing
// TODO: Use auton time estimation to actively choose which is the best/highest score value auton plan given current sensor inputs and game state estimation, best if can recognize opponent bot position
// TOD: AutonSelections screen multi-line √
// TOD: Preauton inert calibration √
// TOD Auton single side turn √
// TODO: Match loader anti-jam
// TODO: All controller with rating and purposes
// TOD Odom use inertial √
// TOD joystick use curve commands(replace) √
// TOD anti-tipping backwards vs fwd √
// TODO: auton plan if teammate does not have auton (score 4 in each of the middle goals to get at most 38 points)
// TOD: fix sballpath functions to add sorting NOT NEEDED RN
// TODO: test balltracker on bot X

// TODO: change accel decel limit
// TOD add screen display curr team √
// TODO: Add hold button
// TOD Add team switch √