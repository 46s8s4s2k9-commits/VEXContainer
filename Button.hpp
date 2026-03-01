// Button.hpp

#ifndef BUTTON_HPP
#define BUTTON_HPP

#include <string>
#include "vex.h"


extern vex::brain Brain;

enum class AutonMode {
    Left,
    Right,
    LeftBall,
    RightBall,
    LeftMatchL,
    RightMatchL
};

class Button {
public:
    AutonMode mode;
    int x;
    int y;
    int w;
    int h;
    std::string text;
    bool selected = false;

    void init(int x, int y, int w, int h, std::string text, AutonMode mode);
    AutonMode getMode();
    void draw();
    bool checkBtnPress();
};

#endif // BUTTON_HPP
