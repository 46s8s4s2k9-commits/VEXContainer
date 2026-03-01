// Lock.cpp

#include "vex.h"
#include "Lock.hpp"

using namespace vex;

Lock::Lock(mutex& l, bool adopted) : lck(l) {
    if (!adopted) {
        lck.lock();
    } 
}

Lock::~Lock() {
    lck.unlock();
}

void Lock::lock() {
    lck.lock();
}
