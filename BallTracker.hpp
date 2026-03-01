// BallTracker.hpp

#ifndef BALLTRACKER_HPP
#define BALLTRACKER_HPP

#include "vex.h"
#include "RobotState.hpp"
#include "SBallPath.hpp"
#include <queue>

extern vex::brain Brain;

enum class ChainColor {
  BLUE,
  RED,
  NONE
};

enum class RemoveBall {
  High,
  Low
};

class BallTracker {
  private:
    int intakeContBallMSec;
    int intakeDebounceTime;
    int outtakeContBallMSec;
    int outtakeDebounceTime;

    bool isUpdating;
    std::queue<ChainColor> pendingBalls;
    std::queue<RemoveBall> pendingRemove;
    vex::mutex pendingRemoveMutex;
    bool processPendRunning;
    vex::mutex processPendRunningMutex;

    vex::thread processPendThread;
    bool processPendStarted;


    bool addBallUnlocked(ChainColor color);
    bool removeBallLowUnlocked();
    bool removeBallHighUnlocked();


  public:
    ChainColor chain[9];
    vex::optical& optIn;
    vex::optical& optOut;
    RobotState& state;
    SBallPath& ballPath;
    
    double optRedMin;
    double optRedMax;
    double optBlueMin;
    double optBlueMax;

    double intakeSlowSpd = 50;
    double outtakeSlowSpd = 50;

    vex::mutex isUpdatingMutex;
    vex::mutex chainMutex;
    vex::mutex pendingBallsMutex;



    BallTracker(vex::optical& optIn, vex::optical& optOut, RobotState& state, SBallPath& ballPath);

    void setChain(ChainColor first, ChainColor second, ChainColor third,
                  ChainColor fourth, ChainColor fifth, ChainColor sixth,
                  ChainColor seventh, ChainColor eighth, ChainColor ninth);
    
    void setIntakeContBallMSec(int mSec);

    void setIntakeDebounceTime(int mSec);

    void setOuttakeContBallMSec(int mSec);

    void setOuttakeDebounceTime(int mSec);

    void setOptRed(double min, double max);

    void setOptBlue(double min, double max);

    void stopUpdate();
    
    void startUpdate();

    bool getUpdateState();

    bool isRed(double value);

    bool isBlue(double value);

    bool addBall(ChainColor color);

    bool addBallSafe(ChainColor color);

    void processPendingBalls();

    void removePendingBallsLow();
    void removePendingBallsHigh();

    bool removeBallLow();
    bool removeBallLowSafe();

    bool removeBallHigh();
    bool removeBallHighSafe();

    int countBalls();

    void clearChain();

    void updateIntake();
    void updateOuttake();

    ChainColor getBallAt(int pos);
    ChainColor getNextBall();
};

#endif
