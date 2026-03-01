//=============================================================================
// PixelText.hpp - Pixel-Perfect Text Rendering System for VEX V5 Brain
// Version: 1.0.1 (Compilation Fix)
// Screen: 480x240 pixels
//=============================================================================

#ifndef PIXELTEXT_HPP
#define PIXELTEXT_HPP

#include "vex.h"
#include <cstdint>
#include <cstdarg>
#include <cstring>
#include <cstdio>

extern vex::brain Brain;

namespace PixelText {

//=============================================================================
// FONT STRUCTURE & DEFINITIONS
//=============================================================================

struct Font {
    const uint8_t* data;
    uint8_t width;
    uint8_t height;
    uint8_t stride;
    char startChar;
    char endChar;
    const char* name;
};

//=============================================================================
// FONT NAMESPACES
//=============================================================================

namespace Font3x5 {
    constexpr uint8_t WIDTH = 3;
    constexpr uint8_t HEIGHT = 5;
    constexpr uint8_t STRIDE = 1;
    constexpr char START = ' ';
    constexpr char END = 'Z';
    
    extern const uint8_t DATA[];
    extern const Font INFO;
}

namespace Font5x7 {
    constexpr uint8_t WIDTH = 5;
    constexpr uint8_t HEIGHT = 7;
    constexpr uint8_t STRIDE = 5;
    constexpr char START = ' ';
    constexpr char END = '~';
    
    extern const uint8_t DATA[];
    extern const Font INFO;
}

namespace Font6x8 {
    constexpr uint8_t WIDTH = 6;
    constexpr uint8_t HEIGHT = 8;
    constexpr uint8_t STRIDE = 6;
    constexpr char START = ' ';
    constexpr char END = '~';
    
    extern const uint8_t DATA[];
    extern const Font INFO;
}

namespace Font7x10 {
    constexpr uint8_t WIDTH = 7;
    constexpr uint8_t HEIGHT = 10;
    constexpr uint8_t STRIDE = 7;
    constexpr char START = ' ';
    constexpr char END = 'Z';
    
    extern const uint8_t DATA[];
    extern const Font INFO;
}

namespace Font8x12 {
    constexpr uint8_t WIDTH = 8;
    constexpr uint8_t HEIGHT = 12;
    constexpr uint8_t STRIDE = 8;
    constexpr char START = ' ';
    constexpr char END = 'Z';
    
    extern const uint8_t DATA[];
    extern const Font INFO;
}

namespace Icon8x8 {
    constexpr uint8_t SIZE = 8;
    
    enum Icons : uint8_t {
        CHECKMARK = 0,
        CROSS,
        ARROW_UP,
        ARROW_DOWN,
        ARROW_LEFT,
        ARROW_RIGHT,
        BATTERY_FULL,
        BATTERY_HALF,
        BATTERY_LOW,
        WARNING,
        INFO,
        SETTINGS,
        PNEUMATIC,
        MOTOR,
        SENSOR,
        RADIO,
        COUNT
    };
    
    extern const uint8_t DATA[][8];
}

//=============================================================================
// RENDERING ENGINE CLASS
//=============================================================================

class Renderer {
private:
    vex::brain& brain;
    
public:
    explicit Renderer(vex::brain& b) : brain(b) {}
    
    // Core drawing
    int drawChar(int x, int y, char c, const PixelText::Font& font, 
                 vex::color fgColor, int bgColor = -1);
    
    int drawString(int x, int y, int maxWidth, const PixelText::Font& font,
                   vex::color fgColor, const char* text);
    
    int drawFormat(int x, int y, const PixelText::Font& font, vex::color fgColor,
                   const char* fmt, ...);
    
    int drawCentered(int centerX, int y, const PixelText::Font& font, 
                     vex::color fgColor, const char* fmt, ...);
    
    int drawRight(int rightX, int y, const PixelText::Font& font,
                  vex::color fgColor, const char* fmt, ...);
    
    void drawIcon(int x, int y, Icon8x8::Icons icon, vex::color color);
    
    int measureText(const char* text, const PixelText::Font& font);
    
    // UI Widgets
    void drawButton(int x, int y, int w, int h, const char* label, uint8_t state = 0);
    
    void drawProgressBar(int x, int y, int w, int h, int value, int maxValue,
                        vex::color fillColor, vex::color bgColor = vex::color(30, 30, 30));
    
    void drawLabelValue(int x, int y, const char* label, const char* value,
                       vex::color labelColor = vex::color(150, 150, 150),
                       vex::color valueColor = vex::color::white);
    
    void drawDivider(int y, vex::color color = vex::color(60, 60, 60));
    
    void drawBox(int x, int y, int w, int h, vex::color fillColor,
                 vex::color borderColor, uint8_t borderWidth = 1);
    
    void clearRegion(int x, int y, int w, int h, 
                    vex::color bgColor = vex::color::black);
    
    void drawStatusDot(int x, int y, int radius, vex::color color);
    
    void drawToggle(int x, int y, int w, int h, bool state, const char* label = nullptr);
    
    void drawGraphPoint(int x, int y, int w, int h, int* values, int count,
                       int minVal, int maxVal, vex::color lineColor);
    
    // Utilities
    void formatBattery(char* buf, size_t size, double voltage);
    void formatTemp(char* buf, size_t size, double temp, bool celsius = true);
    void formatTime(char* buf, size_t size, uint32_t milliseconds);
    
    vex::brain& getBrain() { return brain; }
};

//=============================================================================
// QUICK FUNCTIONS
//=============================================================================

namespace Quick {
    void print5x7(int x, int y, vex::color color, const char* fmt, ...);
    void print6x8(int x, int y, vex::color color, const char* fmt, ...);
    void print7x10(int x, int y, vex::color color, const char* fmt, ...);
    void printCentered(int cx, int y, vex::color color, const char* fmt, ...);
    void printRight(int rx, int y, vex::color color, const char* fmt, ...);
}

//=============================================================================
// COLOR PALETTE
//=============================================================================

namespace Colors {
    // Status colors
    inline const vex::color SUCCESS = vex::color(0, 255, 0);
    inline const vex::color WARNING = vex::color(255, 200, 0);
    inline const vex::color ERROR = vex::color(255, 0, 0);
    inline const vex::color INFO = vex::color(0, 150, 255);
    
    // UI colors
    inline const vex::color BACKGROUND = vex::color(0, 0, 0);
    inline const vex::color PANEL = vex::color(20, 20, 20);
    inline const vex::color BORDER = vex::color(60, 60, 60);
    inline const vex::color TEXT_PRIMARY = vex::color(255, 255, 255);
    inline const vex::color TEXT_SECONDARY = vex::color(150, 150, 150);
    inline const vex::color TEXT_DISABLED = vex::color(80, 80, 80);
    
    // Button states
    inline const vex::color BTN_NORMAL = vex::color(40, 40, 40);
    inline const vex::color BTN_SELECTED = vex::color(0, 120, 255);
    inline const vex::color BTN_PRESSED = vex::color(255, 255, 255);
    
    // Team colors
    inline const vex::color RED_TEAM = vex::color(255, 50, 50);
    inline const vex::color BLUE_TEAM = vex::color(50, 150, 255);
    
    // Graph colors
    inline const vex::color GRAPH_1 = vex::color(0, 200, 255);
    inline const vex::color GRAPH_2 = vex::color(255, 100, 0);
    inline const vex::color GRAPH_3 = vex::color(100, 255, 0);
    inline const vex::color GRAPH_4 = vex::color(255, 0, 200);
}

} // namespace PixelText

#endif // PIXELTEXT_HPP