// PneumaticsTracker.cpp

#include "../include/PneumaticsTracker.hpp"

PneumaticsTracker::PneumaticsTracker() : maxUses(20), currentUses(0) {}


void PneumaticsTracker::setMax(double max) {
    maxUses = max;
}

void PneumaticsTracker::use(double units) {
    currentUses += units;
}

double PneumaticsTracker::remaining() {
    return maxUses - currentUses;
}

void PneumaticsTracker::reset() {
    currentUses = 0;
}

