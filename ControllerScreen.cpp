// ControllerScreen.cpp 

#include "vex.h"
#include "ControllerScreen.hpp"
#include "Lock.hpp"


using namespace vex;

extern bool isSort;

ControllerScreen::ControllerScreen(controller Controller1) : Controller1(Controller1) {}

void ControllerScreen::refresh() {
    Controller1.Screen.clearScreen();

    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Remaining %.1f", pn);

    Controller1.Screen.newLine();
    if (teamIsBlue) {
        Controller1.Screen.print("Team: Blue");
    } else {
        Controller1.Screen.print("Team: Red");
    }
    Controller1.Screen.setCursor(2, 12);
    if (isSort) {
        Controller1.Screen.print("Enable");
    } else {
        Controller1.Screen.print("Disable");
    }
}

void ControllerScreen::setPn(double num) {
    pn = num;
}

void ControllerScreen::setTeam(bool isBlue) {
    teamIsBlue = isBlue;
}