// AutonRoutes.cpp - Autonomous Route Implementations

#include "vex.h"
#include "Robot.hpp"
#include "AutonRoute.hpp"
#include "AutonRoutes.hpp"
#include <vector>
#include "Lock.hpp"

using namespace vex;

// External references to hardware
extern motor LF, LM, LB, RF, RM, RB;
extern motor Intake, Outtake, End;
extern optical Opt, IntakeOpt;
extern inertial Inertial1;
extern digital_out PnLoader, PnSDescore, PnFDescore;
extern brain Brain;
extern controller Controller1;
extern bool isLoaderDown, isSDescoreUp, isFDescoreUp, teamIsBlue;

// External robot instance
extern Robot robot;

double distWrapper;
double monitorPIDDist;


ballPathCommand pathCmd;
mutex pathCmdMutex;

///////////////////////////////////////////////////
// Autonomous Helper Functions
/////////////////////////////////////////////////////////////////////


void ballPathLoop() {
    ballPathCommand cmd;
    int time = 0;
    {
    Lock lock(pathCmdMutex);
    pathCmd = ballPathCommand::None;
    }
    while (true) {
        {
            Lock pathCmdLock(pathCmdMutex);
            cmd = pathCmd;
        }

        if (cmd == ballPathCommand::High) {
            robot.ballPath.highOut();
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

        if (cmd == ballPathCommand::Low) {
            robot.ballPath.lowOut();
        }

        if (cmd == ballPathCommand::Mid) {
            robot.ballPath.midOut(100, 100, 50);
        }

        if (cmd == ballPathCommand::Store) {
            robot.ballPath.storeBalls();
        }

        if (cmd == ballPathCommand::None) {
            robot.ballPath.stop();
        }

        if (cmd == ballPathCommand::Delay) {
            if (time == 0) {
                time = Brain.Timer.time();
                robot.ballPath.highDelay();
            } else if (Brain.Timer.time()-time >= 300) {
                {
                    Lock pathCmdLock(pathCmdMutex);
                    pathCmd = ballPathCommand::High;
                }
                time = 0;
            }
        }
        task::sleep(50);
    }
}

/**
 * @brief Intakes balls until the top optical sensor detects one
 * @param extra Additional time to continue intake after detection (ms)
 * @param max Maximum time to wait for ball detection (ms)
 * @return 0 when complete
 */
int intakeUntilTop(double extra, double max = 3000) {
    robot.ballPath.storeBalls();
    double startTime = Brain.Timer.time(msec);
    while (!Opt.isNearObject() && (Brain.Timer.time(msec) - startTime) < max) {
        task::sleep(20);
    }
    task::sleep(extra);
    robot.ballPath.stop();
    return 0;
}

/**
 * @brief Activates match loader after a delay
 * @param ms Delay before activating loader (milliseconds)
 */
void matchLoaderAuton(double ms) {
    task::sleep(ms);
    PnLoader.set(true);
    return;
}

/**
 * @brief Task wrapper for PID drive forward 60cm
 * @return 0 when complete
 */
void pidDriveWrapper() {
    robot.auton.PIDdriveForward(distWrapper);
}

void storeBallsWrapper(){
    robot.ballPath.storeBalls();
}

void monitorStore(int target, int timeout = 3000) {
    int startT = Brain.Timer.time() + robot.state.trueTimeCorr;
    while (Brain.Timer.time()+robot.state.trueTimeCorr - startT < timeout) {
        if (robot.chain.countBalls() >=target) {
            break;
        }
        task::sleep(50);
    }
}

void monitorOut(int target, int timeout = 3000) {
    int startT = Brain.Timer.time() + robot.state.trueTimeCorr;
    while (Brain.Timer.time()+robot.state.trueTimeCorr - startT < timeout) {
        if (robot.chain.countBalls() <=target) {
            break;
        }
        task::sleep(20);
    }
}


///////////////////////////////////////////////////
// Autonomous Route Implementations
/////////////////////////////////////////////////////////////////////

void OdomleftBallAuton() {
    robot.odometry.reset(136.26, 17.75, 0, robot.driveBase.getLeftEncoder(), robot.driveBase.getRightEncoder());
    robot.autonCommands.moveTo(136.26,90.378, true, 337);
    wait(100, msec);
    distWrapper = 35;
    thread drive1 = thread(pidDriveWrapper);
    while (robot.auton.getDistanceRemaining() > 20) {
        task::sleep(20);
    }
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Store;
    }
    wait(200, msec);
    PnLoader.set(true);
    isLoaderDown = true;
    while (!robot.auton.isPIDOver()) {
        task::sleep(20);
    }
    wait(500, msec);
    robot.autonCommands.turnToHeading(225);
    wait(100, msec);
    robot.autonCommands.moveTo(145.33,145.34, true, 225);
    robot.ballPath.midOut(100, 100, 60);
    wait(5000,msec);
}

/**
 * @brief Basic left side autonomous
 * 
 * Route: Starting position → Drive forward → Turn left → Intake balls → Turn around
 * 
 * Starting position: (137.215, 28.36) facing 0°
 */
void leftAuton() {
    robot.odometry.reset(136.26, 18.75, 0, robot.driveBase.getLeftEncoder(), robot.driveBase.getRightEncoder());
    robot.auton.PIDdriveForward(65);
    robot.auton.PIDturnLeft(23);

    distWrapper = 35;
    thread drive1 = thread(pidDriveWrapper);
    while (robot.auton.getDistanceRemaining() > 25) {
        task::sleep(20);
    }
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Store;
    }
    wait(200, msec);
    PnLoader.set(true);
    isLoaderDown = true;
    wait(10, msec);
    while (!robot.auton.isPIDOver()) {
        task::sleep(20);
    }
    robot.autonCommands.turnToHeading(225);
    robot.auton.PIDdriveForward(-37.16);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Mid;
    }
    wait(550,msec);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::None;
    }
    robot.auton.PIDdriveForward(120.2);
    robot.autonCommands.turnToHeading(180);
    robot.auton.setPIDLimit(30);
    robot.auton.PIDdriveForward(30.5);
    robot.auton.setPIDLimit(60);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Store;
    }
    wait(400, msec);
    robot.auton.PIDdriveForward(-69.22);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Delay;
    }
    PnLoader.set(false);
    isLoaderDown = false;
    wait(1500, msec);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::None;
    }
    PnFDescore.set(true);
    isFDescoreUp = true;
    robot.auton.PIDdriveForward(15);
    PnSDescore.set(true);
    isSDescoreUp = true;
    robot.autonCommands.turnToHeading(270);
    robot.auton.PIDdriveForward(-27);
    robot.autonCommands.turnToHeading(180);
    PnSDescore.set(false);
    isSDescoreUp = false;
    robot.auton.setPIDLimit(60);
    robot.auton.PIDdriveForward(-60);
    robot.driveBase.setHold();
}



/**
 * @brief Left side autonomous with match loader
 * 
 * Route: Starting position → Drive forward → Turn left → Intake balls → 
 *        Deploy match loader → Navigate to loading zone
 * 
 * Starting position: (137.215, 28.36) facing 0°
 */
void leftMatchLoader() {
    robot.odometry.reset(136.26, 17.75, 0, robot.driveBase.getLeftEncoder(), robot.driveBase.getRightEncoder());
    robot.auton.PIDdriveForward(65);
    wait(100, msec);
    robot.auton.PIDturnLeft(23);
    wait(100, msec);
    distWrapper = 35;
    thread drive1 = thread(pidDriveWrapper);
    while (robot.auton.getDistanceRemaining() > 20) {
        task::sleep(20);
    }
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Store;
    }wait(200, msec);
    PnLoader.set(true);
    while (!robot.auton.isPIDOver()) {
        task::sleep(20);
    }
    wait(500, msec);
    robot.autonCommands.turnToHeading(225);
    wait(100, msec);
    robot.auton.PIDdriveForward(90.467);
    wait(100, msec);
    robot.auton.PIDturnLeft(45);
    wait(100,msec);
    robot.auton.PIDdriveForward(36.61);
    robot.ballPath.storeBalls();
    wait(2500,msec);
    robot.auton.PIDdriveForward(-75);
    robot.ballPath.highOut();
    wait(2000,msec);
    robot.ballPath.stop();
    robot.auton.PIDsingTurnL(45);
}


/**
 * @brief Extended right side autonomous
 * 
 * Route: Starting position → Drive forward → Turn right → Intake balls → 
 *        Score in low goal → Return to base → Deploy match loader
 * 
 * Starting position: (219.435, 28.36) facing 0°
 */
void rightAuton() {
    robot.odometry.reset(219.435, 18.75, 0, robot.driveBase.getLeftEncoder(), robot.driveBase.getRightEncoder());
    robot.auton.PIDdriveForward(65);
    robot.auton.PIDturnRight(23);

    distWrapper = 35;
    thread drive1 = thread(pidDriveWrapper);
    while (robot.auton.getDistanceRemaining() > 25) {
        task::sleep(20);
    }
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Store;
    }
    wait(200, msec);
    PnLoader.set(true);
    isLoaderDown = true;
    wait(10, msec);
    while (!robot.auton.isPIDOver()) {
        task::sleep(20);
    }
    robot.autonCommands.turnToHeading(315);
    robot.auton.PIDdriveForward(33.16);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Low;
    }
    wait(550,msec);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::None;
    }
    robot.auton.PIDdriveForward(-122.2);
    robot.autonCommands.turnToHeading(180);
    robot.auton.setPIDLimit(25);
    robot.auton.PIDdriveForward(30.5);
    robot.auton.setPIDLimit(45);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Store;
    }
    wait(400, msec);
    robot.auton.PIDdriveForward(-69.22);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Delay;
    }
    PnLoader.set(false);
    isLoaderDown = false;
    wait(1500, msec);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::None;
    }
    PnFDescore.set(true);
    isFDescoreUp = true;
    robot.auton.PIDsingTurnL(55.25);
    robot.autonCommands.singTurnToHeading(180);
    PnSDescore.set(false);
    isSDescoreUp = false;
    robot.auton.setPIDLimit(50);
    robot.auton.PIDdriveForward(-60);
    robot.driveBase.setHold();
}

/**
 * @brief Right side autonomous with match loader
 * 
 * Route: Starting position → Drive forward → Turn right → Intake balls → 
 *        Deploy match loader → Navigate to loading zone
 * 
 * Starting position: (219.435, 28.36) facing 0°
 */
void rightMatchLoader() {
    robot.odometry.reset(219.435, 28.36, 0, robot.driveBase.getLeftEncoder(), robot.driveBase.getRightEncoder());
    robot.auton.PIDdriveForward(39.5);
    wait(200, msec);
    robot.auton.PIDturnRight(17);
    wait(200, msec);
    {
        Lock lock(pathCmdMutex);
        pathCmd = ballPathCommand::Store;
    }
    robot.auton.PIDdriveForward(60);
    PnLoader.set(true);
    wait(200, msec);
    robot.auton.PIDturnRight(118.87);
    wait(200, msec);
    robot.auton.PIDdriveForward(84.2);
    wait(200, msec);
    robot.auton.PIDturnRight(45);
}

void autonSkills() {
    robot.odometry.reset(178.32, 60.63, 0, robot.driveBase.getLeftEncoder(), robot.driveBase.getRightEncoder());

}

///////////////////////////////////////////////////
// Route Registry
/////////////////////////////////////////////////////////////////////

/**
 * @brief Central definition of all autonomous routes
 * 
 * ADD NEW ROUTES HERE - This is the only place you need to modify to add a new autonomous routine!
 * 
 * Each route needs:
 * - name: Full display name shown on brain screen
 * - shortName: Abbreviated name for compact display
 * - execute: Function that runs the autonomous routine
 * 
 * The routes will automatically appear in selection UI in the order defined here.
 */
std::vector<AutonRoute> getAutonRoutes() {
    return {
        AutonRoute("Left", "L", leftAuton),
        AutonRoute("Right", "R", rightAuton),
        AutonRoute("Left Match Load", "LM", leftMatchLoader),
        AutonRoute("Right Match Load", "RM", rightMatchLoader),
        //AutonRoute("Odom Left Ball", "OLB", OdomleftBallAuton)
    };
}