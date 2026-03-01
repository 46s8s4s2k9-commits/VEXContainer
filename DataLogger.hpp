// DataLogger.hpp

#ifndef DATALOGGER_HPP
#define DATALOGGER_HPP

#include <string>
#include "CurrEvent.hpp" // Assuming CurrEvent is defined in a separate file
#include "vex.h"
#include "SixWheelDrive.hpp"
#include "OdometryTracker.hpp"
#include "PneumaticsTracker.hpp"
#include "RobotState.hpp"

extern vex::brain Brain;


class DataLogger {
public:
    DataLogger();
    bool isLogging;
    int getNextFile();
    void startLogging();
    void stopLogging();
    void logData(SixWheelDrive& driveBase, OdometryTracker& odom, PneumaticsTracker& pn, RobotState& state, CurrEvent currEvent, double joy3, double joy1, double optGrey);
    void displayStats();
    std::string getFilename() const;
  
private:
    struct DataPoint {
        double timestamp;
        CurrEvent currEvent;
        double joyStickInput3;
        double joyStickInput1;
        double leftMotorPower;
        double rightMotorPower;
        double leftMotorTemp;
        double rightMotorTemp;
        double batteryVoltage;
        double batteryCurrent;
        double posX;
        double posY;
        double heading;
        int pneumaticUses;
        double optGrey;
    };

    std::string filename; // filename for this run
    double maxLeftTemp = 0, maxRightTemp = 0;
    double avgLeftPower = 0, avgRightPower = 0;
    double minVoltage = 999;
    int count = 0;

    void writeHeader();
    void appendData(const DataPoint& point);
};

#endif // DATALOGGER_HPP
