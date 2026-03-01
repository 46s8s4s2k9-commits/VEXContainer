// Auton.cpp

#include "Auton.hpp"

using namespace vex;

extern inertial Inertial1;

Auton::Auton(RobotState& state, SixWheelDrive& driveBase, OdometryTracker& odometry, ControlUnit& control) 
    : proportional(0), integral(0), derivative(0), initError(0), 
      turnInitError(0), turnLastError(0), lastTime(0), currTime(0), 
      lastError(0), target(0), turnTarget(0), driveBase(driveBase), odometry(odometry), state(state), control(control) {}

void Auton::setAutonCorrections(double lftCorr, double rghtCorr) {
    autonLftCorr = lftCorr;
    autonRghtCorr = rghtCorr;
}

void Auton::setPIDLimit(double lim) {
    pidLimit = lim;
}

void Auton::setPIDTimeout(double ms) {
    pidTimeout = ms;
}

void Auton::setPIDConstants(double p, double i, double d) {
    kP = p;
    kI = i;
    kD = d;
}

void Auton::setTurnPIDConstants(double p, double i, double d) {
    turnKp = p;
    turnKi = i;
    turnKd = d;
}

double Auton::degToCm(double degree) {
    return (degree / 360.0) * driveBase.gearRatio * driveBase.wheelCirc;
}

double Auton::cmToDeg(double cm) {
    return ((cm / driveBase.wheelCirc) / driveBase.gearRatio) * 360.0;
}

void Auton::resetAction() {
    target = 0;
    initError = 0;
    lastError = 0;
    proportional = 0;
    integral = 0;
    derivative = 0;

    lastTime = 0;
    turnTarget = 0;
    turnInitError = 0;
    turnLastError = 0;
}

double Auton::dynamicKI(double maxKi, double minKi, double maxError, double currError) {
    if (maxError == 0) {
        maxError += 0.01;
    }
    double newKi;
    if (currError > maxError) {
        newKi = (minKi + maxKi) / 2;
    } else {
        newKi = minKi + ((maxKi - minKi) * (1 - fabs((currError / maxError))));
    }
    if (newKi > maxKi) newKi = maxKi;
    if (newKi < minKi) newKi = minKi;
    return newKi;
}


void Auton::PIDdriveForward(double dist, bool timeOut) {
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = false;
    }
    {
        Lock autonStateLock(autonStateMutex);
        if (dist < 0) {
            autonState = AutonState::Reverse;
        } else {
            autonState = AutonState::Forward;
        }
    }

    double startLeft = driveBase.getLeftEncoder();
    double startRight = driveBase.getRightEncoder();
    double startTime = Brain.Timer.time(msec) + state.trueTimeCorr;

    double o1 = 999;
    bool atO2 = false;
    double o2;
    
    double startRotation;
    if (trustInert) {
        startRotation = state.getRotation();
    }
    
    resetAction();

    target = dist;
    double error;
    int settledCount = 0;
    const int SETTLED_ITERATIONS = 5;

    lastTime = Brain.Timer.time(msec);
    
    while (true) {
        if (timeOut) {
            if ((Brain.Timer.time(msec) + state.trueTimeCorr) - startTime > pidTimeout) {
                Brain.Screen.printAt(1, 20, "PID TIMEOUT!");
                break;
            }
        }
        double lPos = degToCm(driveBase.getLeftEncoder() - startLeft);
        double rPos = degToCm(driveBase.getRightEncoder() - startRight);
        double pos = (lPos + rPos) / 2.0;
        error = target - pos;

        if (o1 == 999) {
            o1 = error;
        } else if (!atO2) {
            if (error <= o1) {
                o1 = error;
            } else {
                atO2 = true;
            }
        } else {
            if (error >= o2) {
                o2 = error;
            }
        }

        if (fabs(error) < 1.0) {
            settledCount++;
            if (settledCount >= SETTLED_ITERATIONS) break;
        } else {
            settledCount = 0;
        }

        double pwr = PID(error, kP, kI, kD);
        if (trustInert) {
            double corr = startRotation - state.getRotation();
            if (dist > 0) {
                drivePct(pwr + corr*pidDriveFwdCorr, pwr - corr*pidDriveFwdCorr);
            } else {
                drivePct(pwr - corr*pidDriveFwdCorr, pwr + corr*pidDriveFwdCorr);
            }
        } else {
            drivePct(pwr, pwr);
        }
        task::sleep(20);
    }

    driveBase.allStop();
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = true;
    }
    {
        Lock autonStateLock(autonStateMutex);
        autonState = AutonState::None;
    }
    AutoTuneFeedback feedback;
    feedback.o1 = o1;
    feedback.o2 = o2;
    feedback.atO2 = atO2;
}

void Auton::PIDturnRight(double deg) {
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = false;
    }
    {
        Lock autonStateLock(autonStateMutex);
        autonState = AutonState::TurnRight;
    }
    
    double startLeft = driveBase.getLeftEncoder();
    double startRight = driveBase.getRightEncoder();
    double startTime = Brain.Timer.time(msec) + state.trueTimeCorr;

    double targetRotation;
    if (trustInert) {
        targetRotation = state.getRotation() + deg;
    }

    double o1 = 999;
    bool atO2 = false;
    double o2;
    
    resetAction();

    double turnDist = (deg / 360.0) * M_PI * driveBase.baseWidth;
    turnTarget = turnDist;
    
    int settledCount = 0;
    const int SETTLED_ITERATIONS = 5;
    lastTime = Brain.Timer.time(msec);
    
    while (true) {
        if ((Brain.Timer.time(msec) + state.trueTimeCorr) - startTime > pidTimeout) {
            Brain.Screen.printAt(1, 20, "PID TIMEOUT!");
            break;
        }
        double lPos = degToCm(driveBase.getLeftEncoder() - startLeft);
        double lError = turnTarget - lPos;
        if (trustInert) {
            double degLeft = targetRotation - state.getRotation();
            double distLeft = (degLeft / 360.0) * M_PI * driveBase.baseWidth;
            lError = distLeft;
        }
        if (o1 == 999) {
            o1 = lError;
        } else if (!atO2) {
            if (lError <= o1) {
                o1 = lError;
            } else {
                atO2 = true;
            }
        } else {
            if (lError >= o2) {
                o2 = lError;
            }
        }
        if (fabs(lError) < 0.5) {
            settledCount++;
            if (settledCount >= SETTLED_ITERATIONS) break;
        } else {
            settledCount = 0;
        }

        double pwr = PID(lError, turnKp, turnKi, turnKd);
        Auton::drivePct(pwr, -pwr);
        task::sleep(20);
    }

    driveBase.allStop();
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = true;
    }
    {
        Lock autonStateLock(autonStateMutex);
        autonState = AutonState::None;
    }
    AutoTuneFeedback feedback;
    feedback.o1 = o1;
    feedback.o2 = o2;
    feedback.atO2 = atO2;
}


void Auton::PIDsingTurnR(double deg) {
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = false;
    }
    {
        Lock autonStateLock(autonStateMutex);
        autonState = AutonState::SingTurnRight;
    }
    
    // Store starting encoder values instead of resetting
    double startLeft = driveBase.getLeftEncoder();
    double startRight = driveBase.getRightEncoder();
    double startTime = Brain.Timer.time(msec) + state.trueTimeCorr;

    double targetRotation;
    if (trustInert) {
        targetRotation = state.getRotation() + deg;
    }
    
    double o1 = 999;
    bool atO2 = false;
    double o2;

    resetAction();

    double turnDist = (deg / 180.0) * M_PI * driveBase.baseWidth;
    turnTarget = turnDist;
    
    int settledCount = 0;
    const int SETTLED_ITERATIONS = 5;
    lastTime = Brain.Timer.time(msec);
    
    while (true) {
        if ((Brain.Timer.time(msec) + state.trueTimeCorr) - startTime > pidTimeout) {
            Brain.Screen.printAt(1, 20, "PID TIMEOUT!");
            break;
        }
        double lPos = degToCm(driveBase.getLeftEncoder() - startLeft);
        double lError = turnTarget - lPos;
        if (trustInert) {
            double degLeft = targetRotation - state.getRotation();
            double distLeft = (degLeft / 180.0) * M_PI * driveBase.baseWidth;
            lError = distLeft;
        }
        if (o1 == 999) {
            o1 = lError;
        } else if (!atO2) {
            if (lError <= o1) {
                o1 = lError;
            } else {
                atO2 = true;
            }
        } else {
            if (lError >= o2) {
                o2 = lError;
            }
        }

        if (fabs(lError) < 0.5) {
            settledCount++;
            if (settledCount >= SETTLED_ITERATIONS) break;
        } else {
            settledCount = 0;
        }

        double pwr = PID(lError, turnKp*1.5, turnKi, turnKd);
        Auton::drivePct(pwr, 0);
        task::sleep(20);
    }

    driveBase.allStop();
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = true;
    }
    {
        Lock autonStateLock(autonStateMutex);
        autonState = AutonState::None;
    }
    AutoTuneFeedback feedback;
    feedback.o1 = o1;
    feedback.o2 = o2;
    feedback.atO2 = atO2;
}

void Auton::PIDturnLeft(double deg) {
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = false;
    }
    {
        Lock autonStateLock(autonStateMutex);
        autonState = AutonState::TurnLeft;
    }
    
    double startLeft = driveBase.getLeftEncoder();
    double startRight = driveBase.getRightEncoder();
    double startTime = Brain.Timer.time(msec) + state.trueTimeCorr;

    double targetRotation;
    if (trustInert) {
        targetRotation = state.getRotation() - deg;
    }
    double o1 = 999;
    bool atO2 = false;
    double o2;
    
    resetAction();

    double turnDist = (deg / 360.0) * M_PI * driveBase.baseWidth;
    turnTarget = turnDist;
    
    int settledCount = 0;
    const int SETTLED_ITERATIONS = 5;
    lastTime = Brain.Timer.time(msec);
    
    while (true) {
        if ((Brain.Timer.time(msec) + state.trueTimeCorr) - startTime > pidTimeout) {
            Brain.Screen.printAt(1, 20, "PID TIMEOUT!");
            break;
        }
        double rPos = degToCm(driveBase.getRightEncoder() - startRight);
        double rError = turnTarget - rPos;
        if (trustInert) {
            double degLeft = -(targetRotation - state.getRotation());
            double distLeft = (degLeft / 360.0) * M_PI * driveBase.baseWidth;
            rError = distLeft;
        }
        if (o1 == 999) {
            o1 = rError;
        } else if (!atO2) {
            if (rError <= o1) {
                o1 = rError;
            } else {
                atO2 = true;
            }
        } else {
            if (rError >= o2) {
                o2 = rError;
            }
        }

        if (fabs(rError) < 0.5) {
            settledCount++;
            if (settledCount >= SETTLED_ITERATIONS) break;
        } else {
            settledCount = 0;
        }

        double pwr = PID(rError, turnKp, turnKi, turnKd);
        drivePct(-pwr, pwr);
        task::sleep(20);
    }

    driveBase.allStop();
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = true;
    }
    {
        Lock autonStateLock(autonStateMutex);
        autonState = AutonState::None;
    }
    AutoTuneFeedback feedback;
    feedback.o1 = o1;
    feedback.o2 = o2;
    feedback.atO2 = atO2;
}

void Auton::PIDsingTurnL(double deg) {
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = false;
    }
    {
        Lock autonStateLock(autonStateMutex);
        autonState = AutonState::SingTurnLeft;
    }
    
    double startLeft = driveBase.getLeftEncoder();
    double startRight = driveBase.getRightEncoder();
    double startTime = Brain.Timer.time(msec) + state.trueTimeCorr;

    double targetRotation;
    if (trustInert) {
        targetRotation = state.getRotation() - deg;
    }
    double o1 = 999;
    bool atO2 = false;
    double o2;
    
    resetAction();

    double turnDist = (deg / 180.0) * M_PI * driveBase.baseWidth;
    turnTarget = turnDist;
    
    int settledCount = 0;
    const int SETTLED_ITERATIONS = 5;
    lastTime = Brain.Timer.time(msec);
    
    while (true) {
        if ((Brain.Timer.time(msec) + state.trueTimeCorr) - startTime > pidTimeout) {
            Brain.Screen.printAt(1, 20, "PID TIMEOUT!");
            break;
        }
        double rPos = degToCm(driveBase.getRightEncoder() - startRight);
        double rError = turnTarget - rPos;
        if (trustInert) {
            double degLeft = -(targetRotation - state.getRotation());
            double distLeft = (degLeft / 180.0) * M_PI * driveBase.baseWidth;
            rError = distLeft;
        }
        if (o1 == 999) {
            o1 = rError;
        } else if (!atO2) {
            if (rError <= o1) {
                o1 = rError;
            } else {
                atO2 = true;
            }
        } else {
            if (rError >= o2) {
                o2 =rError;
            }
        }

        if (fabs(rError) < 0.5) {
            settledCount++;
            if (settledCount >= SETTLED_ITERATIONS) break;
        } else {
            settledCount = 0;
        }

        double pwr = PID(rError, turnKp*1.5, turnKi, turnKd);
        drivePct(0, pwr);
        task::sleep(20);
    }

    driveBase.allStop();
    {
        Lock pidOverLock(pidOverMutex);
        pidOver = true;
    }
    {
        Lock autonStateLock(autonStateMutex);
        autonState = AutonState::None;
    }
    AutoTuneFeedback feedback;
    feedback.o1 = o1;
    feedback.o2 = o2;
    feedback.atO2 = atO2;
}

double Auton::PID(double error, double kp, double ki, double kd) { 
    currTime = Brain.Timer.time(msec);
    double lapseTime = (currTime - lastTime) / 1000.0;
    if (lapseTime > 0.1) lapseTime = 0.02;

    lastTime = currTime;

    proportional = 0;
    derivative = 0;

    proportional = error;
    
    if (fabs(error) < 30) {
        integral += error * lapseTime; // sec

        if (integral > 100) {
            integral = 100;
        }
        if (integral < -100) {
            integral = -100;
        }
    } else {
        integral = 0;
    }

    if (lapseTime != 0) {
        Lock lock(lastErrorMutex);
        derivative = (error - lastError) / lapseTime;
    } else {
        derivative = 0;
    } 
    
    {
        Lock lock(lastErrorMutex);
        lastError = error;
    }

    double pwr;

    if (integral * ki > 80) {
        pwr = proportional * kp + 80 + derivative * kd;
    } else if (integral * ki < -80) {
        pwr = proportional * kp - 80 + derivative * kd;
    } else {
        pwr = proportional * kp + integral * ki + derivative * kd;
    }

    if (pwr > pidLimit) pwr = pidLimit;
    if (pwr < -pidLimit) pwr = -pidLimit;

    return pwr;
}

double Auton::getCurrentError() {
    return lastError;
}

double Auton::getCurrentTarget() {
    return target;
}

// !!! Not usable
double Auton::getTurnError() {
    return turnLastError;
}

double Auton::getTurnTarget() {
    return turnTarget;
}

double Auton::getDistanceRemaining() {
    Lock lock(lastErrorMutex);
    return fabs(lastError);
}

// !!! Not usable
double Auton::getTurnDegreesRemaining() {
    double turnDist = fabs(turnLastError);
    return (turnDist / (M_PI * driveBase.baseWidth)) * 360.0;
}

void Auton::drivePct(double leftSpd, double rightSpd) {
    double lftCorrected = leftSpd * autonLftCorr;
    double rghtCorrected = rightSpd * autonRghtCorr;

    double maxAbs = fmax(fabs(lftCorrected), fabs(rghtCorrected));
    
    if (maxAbs > pidLimit) {
        double scale = pidLimit / maxAbs;
        lftCorrected *= scale;
        rghtCorrected *= scale;
    }

    if (lftCorrected > pidLimit) lftCorrected = pidLimit;
    if (lftCorrected < -pidLimit) lftCorrected = -pidLimit;
    if (rghtCorrected > pidLimit) rghtCorrected = pidLimit;
    if (rghtCorrected < -pidLimit) rghtCorrected = -pidLimit;

    if (useControl) {
        control.controlledDrive(lftCorrected, rghtCorrected, true);
    } else {
        driveBase.lrDrive(lftCorrected, rghtCorrected);
    }
}

bool Auton::isPIDOver() {
    Lock pidOverLock(pidOverMutex);
    return pidOver;
}

AutonState Auton::getAutonState() {
    Lock autonStateLock(autonStateMutex);
    return autonState;
}

void Auton::setPIDDriveFwdCorr(double index) {
    pidDriveFwdCorr = index;
}

void Auton::setTrustInert(bool trust) {
    Lock lock(trustInertMutex);
    trustInert = trust;
}

void Auton::setUseControl(bool control) {
    useControl = control;
}

void Auton::transition2Leg(double leg1, TransitionWPT pt2) {

}

void Auton::transition3Leg(double leg1, TransitionWPT pt2, TransitionWPT pt3) {

}
