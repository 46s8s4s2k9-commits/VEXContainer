// Button.cpp

#include "../include/Button.hpp"

// Screen is 479 x 239

/**
 * Initializes a button object with the given parameters.
 * @param x The x-coordinate of the button's top-left corner.
 * @param y The y-coordinate of the button's top-left corner.
 * @param w The width of the button.
 * @param h The height of the button.
 * @param text The text to display on the button.
 * @param mode The auton mode to associate with the button.
 */
void Button::init(int x, int y, int w, int h, std::string text, AutonMode mode) {
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    this->text = text;
    this->mode = mode;
}

/**
 * @brief Returns the auton mode associated with this button.
 * @return The auton mode associated with this button.
 */
AutonMode Button::getMode() {
    return mode;
}

/**
 * @brief Draws the button on the screen.
 * If the button is selected, it will be drawn in green.
 * Otherwise, it will be drawn in blue.
 * The button's text will be drawn in white at the center of the button.
 */
void Button::draw() {
    if (selected) {
    Brain.Screen.setFillColor(vex::green);
    } else {
    Brain.Screen.setFillColor(vex::blue);
    }
    Brain.Screen.setPenColor(vex::white);
    Brain.Screen.drawRectangle(x, y, w, h);
    Brain.Screen.setPenColor(vex::white);
    int txtLen = text.length();
    int midX = x + (w/2);
    int midY = y + (h/2);
    int colW = round(479.0/48.0);
    int rowH = round(239.0/12.0);
    int txtX = round((midX - round(txtLen/2.0) * colW) / (double)colW);
    int txtY = round(midY / (double)rowH);
    Brain.Screen.setCursor(txtY, txtX);
    const char* cstr = text.c_str();
    Brain.Screen.print(cstr);
}

/**
 * @brief Checks if the button has been pressed.
 * @return True if the button has been pressed, false otherwise.
 * 
 * This function checks if the screen is being pressed, and if the 
 * coordinates of the press are within the bounds of the button.
 */
bool Button::checkBtnPress() {
    // FIXED: Added Brain.Screen.pressing() check
    if (Brain.Screen.pressing() && 
        Brain.Screen.xPosition() >= x && 
        Brain.Screen.xPosition() <= x+w && 
        Brain.Screen.yPosition() >= y && 
        Brain.Screen.yPosition() <= y+h) {
    return true;
    }
    return false;
}

