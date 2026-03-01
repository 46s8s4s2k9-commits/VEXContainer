// BallTracker.cpp

#include "BallTracker.hpp"
#include "Lock.hpp"

using namespace vex;

/**
 * @brief Constructs a BallTracker object.
 *
 * This constructor constructs a BallTracker object by
 * clearing the chain state.
 */
BallTracker::BallTracker(vex::optical& optIn, vex::optical& optOut, RobotState& state, SBallPath& ballPath) : optIn(optIn), optOut(optOut), state(state), ballPath(ballPath) {
    clearChain();
}



bool BallTracker::isRed(double value) {
  bool isOverZero;
  if ((optRedMax - optRedMin) < 0) {
    isOverZero = true;
  } else {
    isOverZero = false;
  }
  if (isOverZero) {
    if (value > optRedMin || value < optRedMax) {
      return true;
    } else {
      return false;
    }
  } else {
    if (value > optRedMin && value < optRedMax) {
      return true;
    } else {
      return false;
    }
  }
}

bool BallTracker::isBlue(double value) {
  bool isOverZero;
  if ((optBlueMax - optBlueMin) < 0) {
    isOverZero = true;
  } else {
    isOverZero = false;
  }
  if (isOverZero) {
    if (value > optBlueMin || value < optBlueMax) {
      return true;
    } else {
      return false;
    }
  } else {
    if (value > optBlueMin && value < optBlueMax) {
      return true;
    } else {
      return false;
    }
  }
}




// TOD Finish, add roller monitoring class first √
void BallTracker::updateIntake() {
  if (intakeDebounceTime > intakeContBallMSec) {
    intakeDebounceTime = intakeContBallMSec/3;
  }
  while (true) {
    while (isUpdating) {
      if (state.ballPathState == BallPathState::None) {
        task::sleep(50);
        continue;
      }
      if (optIn.isNearObject()) {
        double hue = optIn.hue();
        if (isRed(hue)) {
          if (fabs(ballPath.getIntakeSpd()) < intakeSlowSpd) {
            if (state.ballPathState != BallPathState::Low) {
              addBallSafe(ChainColor::RED);
            } else {
              removeBallLowSafe();
            }
            while (optIn.isNearObject()) {
              task::sleep(20);
            }
            continue;
          }
          task::sleep(intakeDebounceTime);
          if (isRed(optIn.hue())) {
            while (isRed(optIn.hue()) && optIn.isNearObject()) {
              if (state.ballPathState != BallPathState::Low) {
                addBallSafe(ChainColor::RED);
              } else {
                removeBallLowSafe();
              }
              task::sleep(intakeContBallMSec-intakeDebounceTime);
            }
          }
        }
        if (isBlue(hue)) {
          if (fabs(ballPath.getIntakeSpd()) < intakeSlowSpd) {
            if (state.ballPathState != BallPathState::Low) {
              addBallSafe(ChainColor::BLUE);
            } else {
              removeBallLowSafe();
            }
            while (optIn.isNearObject()) {
              task::sleep(20);
            }
            continue;
          }
          task::sleep(intakeDebounceTime);
          if(isBlue(optIn.hue())) {
            while (isBlue(optIn.hue()) && optIn.isNearObject()) {
              if (state.ballPathState != BallPathState::Low) {
                addBallSafe(ChainColor::BLUE);
              } else {
                removeBallLowSafe();
              }
              task::sleep(intakeContBallMSec-intakeDebounceTime);
            }
          }
        }
      }
      task::sleep(20);
    }
    task::sleep(20);
  }
}

void BallTracker::updateOuttake() {
  if (outtakeDebounceTime > outtakeContBallMSec) {
    outtakeDebounceTime = outtakeContBallMSec/3;
  }
  while (true) {
    while (isUpdating) {
      if (state.ballPathState == BallPathState::None) {
        task::sleep(50);
        continue;
      }
      if (optOut.isNearObject()) {
        if (fabs(ballPath.getEndSpd()) < outtakeSlowSpd) {
          if (state.ballPathState != BallPathState::Store && state.ballPathState != BallPathState::Low) {
            removeBallHighSafe();
          }
          while (optOut.isNearObject()) {
            task::sleep(20);
          }
          continue;
        }
        while (optOut.isNearObject()) {
          if (state.ballPathState != BallPathState::Store && state.ballPathState != BallPathState::Low) {
            removeBallHighSafe();
          }
          task::sleep(outtakeContBallMSec - outtakeDebounceTime);
        }
      }
      task::sleep(20);
    }
    task::sleep(20);
  }
}

static void processPendingBallsWrapper(void* arg) {
  BallTracker* tracker = static_cast<BallTracker*>(arg);
  tracker->processPendingBalls();
}


/**
 * @brief Sets the chain state of the ball tracker object.
 *
 * This function sets the chain state of the ball tracker object
 * by setting the values of the first, second, third, fourth,
 * fifth, sixth, seventh, eighth, and ninth chain colors.
 *
 * @param first The first chain color.
 * @param second The second chain color.
 * @param third The third chain color.
 * @param fourth The fourth chain color.
 * @param fifth The fifth chain color.
 * @param sixth The sixth chain color.
 * @param seventh The seventh chain color.
 * @param eighth The eighth chain color.
 * @param ninth The ninth chain color.
 */
void BallTracker::setChain(ChainColor first, ChainColor second, ChainColor third,
                           ChainColor fourth, ChainColor fifth, ChainColor sixth,
                           ChainColor seventh, ChainColor eighth, ChainColor ninth) {
  Lock chainLock(chainMutex);
  chain[0] = first;
  chain[1] = second;
  chain[2] = third;
  chain[3] = fourth;
  chain[4] = fifth;
  chain[5] = sixth;
  chain[6] = seventh;
  chain[7] = eighth;
  chain[8] = ninth;
}

/**
 * @brief Sets the time in milliseconds for the continuous ball counter.
 * 
 * @param mSec the time in milliseconds for the continuous ball counter.
 */
void BallTracker::setIntakeContBallMSec(int mSec) {
  intakeContBallMSec = mSec;
}

void BallTracker::setIntakeDebounceTime(int mSec) {
  if (mSec < 0) {
    mSec = -mSec;
  }
  if (mSec > intakeContBallMSec) {
    mSec = 0;
  }
  intakeDebounceTime = mSec;
}

void BallTracker::setOuttakeDebounceTime(int mSec) {
  if (mSec < 0) {
    mSec = -mSec;
  }
  if (mSec > outtakeContBallMSec) {
    mSec = 0;
  }
  outtakeDebounceTime = mSec;
}

void BallTracker::setOuttakeContBallMSec(int mSec) {
  outtakeContBallMSec = mSec;
}

void BallTracker::setOptRed(double min, double max) {
  optRedMin = min;
  optRedMax = max;
}

void BallTracker::setOptBlue(double min, double max) {
  optBlueMin = min;
  optBlueMax = max;
}



void BallTracker::stopUpdate() {
  Lock isUpdatingLock(isUpdatingMutex);
  isUpdating = false;
}

void BallTracker::startUpdate() {
  Lock isUpdatingLock(isUpdatingMutex);
  isUpdating = true;
}

bool BallTracker::getUpdateState() {
  Lock isUpdatingLock(isUpdatingMutex);
  return isUpdating;
}

/**
 * @brief Adds a ball to the ball tracker chain.
 * 
 * This function adds a ball of the given color to the ball tracker chain.
 * If there is already a ball in the chain with the given color,
 * the function does nothing and returns false. Otherwise, the function
 * adds the ball to the chain and returns true.
 * 
 * @param color The color of the ball to be added.
 * @return True if the ball was added to the chain, false otherwise.
 */
bool BallTracker::addBall(ChainColor color) {
  Lock chainLock(chainMutex);
  return addBallUnlocked(color);
}

bool BallTracker::addBallUnlocked(ChainColor color) {
  for(int i = 0; i < 9; i++) {
    if(chain[i] == ChainColor::NONE) {
      chain[i] = color;
      return true;
    }
  }
  return false;
}

bool BallTracker::addBallSafe(ChainColor color) {
  bool a = true;
  if (chainMutex.try_lock()) {
    a = addBallUnlocked(color);
    chainMutex.unlock();
  } else {
    {
      Lock pendingBallsLock(pendingBallsMutex);
      pendingBalls.push(color);
    }
    bool shouldStartThread = false;
    {
      Lock lock(processPendRunningMutex);
      if (!processPendRunning) {
        processPendRunning = true;
        shouldStartThread = true;
      }
    }
    if (shouldStartThread) {
      if (processPendStarted && processPendThread.joinable()) {
        processPendThread.detach();
      }
      processPendThread = thread(processPendingBallsWrapper, this);
      processPendThread.setPriority(14);
      processPendStarted = true;
    }
  }
  return a;
}

void BallTracker::processPendingBalls() {
  while (true) {
    bool hasWork = false;
    bool isAdd = false;
    ChainColor addColor = ChainColor::NONE;
    RemoveBall removeType = RemoveBall::Low;

    {
      Lock pendingBallsLock(pendingBallsMutex);
      Lock pendingRemoveLock(pendingRemoveMutex);

      if (!pendingBalls.empty() || !pendingRemove.empty()) {
        hasWork = true;
        if (!pendingBalls.empty() && pendingBalls.front() != ChainColor::NONE) {
          isAdd = true;
          addColor = pendingBalls.front();
          pendingBalls.pop();
        } else if (!pendingBalls.empty()) {
          pendingBalls.pop();
          if (!pendingRemove.empty()) {
            removeType = pendingRemove.front();
            pendingRemove.pop();
          }
        }
      }
    }

    if (!hasWork) {
      break;
    }

    {
      Lock chainLock(chainMutex);
      if (isAdd) {
        addBallUnlocked(addColor);
      } else {
        if (removeType == RemoveBall::High) {
          removeBallHighUnlocked();
        } else {
          removeBallLowUnlocked();
        }
      }
    }

    task::sleep(10);  
  }

  {
    Lock lock(processPendRunningMutex);
    processPendRunning = false;
    processPendStarted = false;
  }
}

/**
 * @brief Removes a ball from the ball tracker chain.
 * 
 * This function removes a ball from the ball tracker chain. If there is
 * already a ball in the chain with the given color, the function
 * does nothing and returns false. Otherwise, the function
 * removes the ball from the chain and returns true.
 * 
 * @return True if the ball was removed from the chain, false otherwise.
 */
bool BallTracker::removeBallLow() {
  Lock chainLock(chainMutex);
  return removeBallLowUnlocked();
}

bool BallTracker::removeBallLowUnlocked() {
  for(int i = 8; i >= 0; i--) {
    if(chain[i] != ChainColor::NONE) {
      chain[i] = ChainColor::NONE;
      return true;
    }
  }
  return false;
}

bool BallTracker::removeBallLowSafe() {
  bool a = true;
  if (chainMutex.try_lock()) {
    a = removeBallLowUnlocked();
    chainMutex.unlock();
  } else {
    {
      Lock lock(pendingBallsMutex);
      pendingBalls.push(ChainColor::NONE);
    }
    {
      Lock lock(pendingRemoveMutex);
      pendingRemove.push(RemoveBall::Low);
    }
    bool shouldStartThread = false;
    {
      Lock lock(processPendRunningMutex);
      if (!processPendRunning) {
        processPendRunning = true;
        shouldStartThread = true;
      }
    }
    if (shouldStartThread) {
      if (processPendStarted && processPendThread.joinable()) {
        processPendThread.detach();
      }
      processPendThread = thread(processPendingBallsWrapper, this);
      processPendThread.setPriority(14);
      processPendStarted = true;
    }
  }
  return a;
}

bool BallTracker::removeBallHigh() {
  Lock chainLock(chainMutex);
  return removeBallHighUnlocked();
}

bool BallTracker::removeBallHighUnlocked() {
  if (chain[0] == ChainColor::NONE) {
    return false;
  }
  chain[0] = chain[1];
  chain[1] = chain[2];
  chain[2] = chain[3];
  chain[3] = chain[4];
  chain[4] = chain[5];
  chain[5] = chain[6];
  chain[6] = chain[7];
  chain[7] = chain[8];
  chain[8] = ChainColor::NONE;
  return true;
}

bool BallTracker::removeBallHighSafe() {
  bool a = true;
  if (chainMutex.try_lock()) {
    a = removeBallHighUnlocked();
    chainMutex.unlock();
  } else {
    {
      Lock lock(pendingBallsMutex);
      pendingBalls.push(ChainColor::NONE);
    }
    {
      Lock lock(pendingRemoveMutex);
      pendingRemove.push(RemoveBall::High);
    }
    bool shouldStartThread = false;
    {
      Lock lock(processPendRunningMutex);
      if (!processPendRunning) {
        processPendRunning = true;
        shouldStartThread = true;
      }
    }
    if (shouldStartThread) {
      if (processPendStarted && processPendThread.joinable()) {
        processPendThread.detach();
      }
      processPendThread = thread(processPendingBallsWrapper, this);
      processPendThread.setPriority(14);
      processPendStarted = true;
    }
  }
  return a;
}


/**
 * @brief Counts the number of balls in the ball tracker chain.
 * 
 * This function counts the number of balls in the ball tracker chain and
 * returns the count. The count is calculated by checking each ball in
 * the chain and incrementing the count if the ball is not of color NONE.
 * 
 * @return The count of balls in the ball tracker chain.
 */
int BallTracker::countBalls() {
  int count = 0;
  Lock chainLock(chainMutex);
  for (int i = 0; i < 9; i++) {
    if (chain[i] != ChainColor::NONE) {
      count++;
    }
  }
  return count;
}

/**
 * @brief Clears the ball tracker chain.
 * 
 * This function clears the ball tracker chain by setting all the chain
 * colors to NONE.
 */
void BallTracker::clearChain() {
  Lock chainLock(chainMutex);
  for (int i = 0; i < 9; i++) {
    chain[i] = ChainColor::NONE;
  }
}


ChainColor BallTracker::getBallAt(int pos) {
  if (pos == 0) {
    return ChainColor::NONE;
  }
  Lock chainLock(chainMutex);
  if (pos >= sizeof(chain)/sizeof(chain[0])) {
    return ChainColor::NONE;
  }
  return chain[pos-1];
}

ChainColor BallTracker::getNextBall() {
  Lock chainLock(chainMutex);
  return chain[0];
}

