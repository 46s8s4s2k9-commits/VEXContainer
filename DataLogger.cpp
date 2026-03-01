// DataLogger.cpp

#include "DataLogger.hpp"
#include "OdometryTracker.hpp"


using namespace vex;

DataLogger::DataLogger() : isLogging(false) {}

// IDEA 3: SD Card Data Logging System

/**
 * Writes the header line to the SD card file.
 *
 * This function checks if the file already exists on the SD card,
 * and if not, writes the header line to the file. The header
 * line contains the names of the data columns in the CSV file.
 */
void DataLogger::writeHeader() {
    if (!Brain.SDcard.exists(filename.c_str())) {
        std::string header =
            "Time,CurrEvent,Joy3,Joy1,LMotor,RMotor,LTemp,RTemp,Voltage,Current,X,Y,Heading,PnUses,OptGrey\n";
        Brain.SDcard.savefile(filename.c_str(), reinterpret_cast<uint8_t*>(const_cast<char*>(header.c_str())), header.length());
    }
}

/**
 * Appends a single data point to the SD card file.
 *
 * This function takes a DataPoint object and appends it to the
 * SD card file specified by the filename. The data point is
 * formatted as a CSV line, with the same order as the header
 * line written by writeHeader. If the file does not exist, it is
 * created. If the file already exists, the data point is appended
 * to the end of the file. If there is a formatting error, the
 * write is skipped.
 */
void DataLogger::appendData(const DataPoint& point) {
    char line[320];  // increased buffer for safety margin

    int len = snprintf(
        line,
        sizeof(line),
        "%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f\n",
        point.timestamp,
        static_cast<int>(point.currEvent),
        point.joyStickInput3,
        point.joyStickInput1,
        point.leftMotorPower,
        point.rightMotorPower,
        point.leftMotorTemp,
        point.rightMotorTemp,
        point.batteryVoltage,
        point.batteryCurrent,
        point.posX,
        point.posY,
        point.heading,
        point.pneumaticUses,
        point.optGrey
    );

    // snprintf returns number of chars that *would* have been written
    if (len < 0) {
        // Formatting error — skip write
        return;
    }

    // If truncated, clamp to buffer size - 1
    if (len >= (int)sizeof(line)) {
        len = sizeof(line) - 2;  // Leave room for \n and \0
        line[len] = '\n';
        line[len + 1] = '\0';
    }

    Brain.SDcard.appendfile(
        filename.c_str(),
        reinterpret_cast<uint8_t*>(line),
        len
    );
}




/**
 * Returns the next available file number on the SD card.
 *
 * This function searches for the next available file number by
 * checking if a file with the name "match_log_<number>.csv"
 * exists on the SD card. If the file exists, it increments
 * the number and checks again. If the file does not exist, it
 * returns the current number.
 *
 * @return The next available file number on the SD card.
 */
int DataLogger::getNextFile() {
int currNum = 1;
bool isFound = false;
char fileName[32];
while (!isFound) {
    sprintf(fileName, "match_log_%d.csv", currNum);
    if (Brain.SDcard.exists(fileName)) {
    currNum++;
    } else {
    isFound = true;
    }
}
return currNum;
}

/**
 * Starts data logging to the SD card.
 *
 * This function starts a new data logging session to the SD card.
 * It generates a unique filename for this run by incrementing a
 * counter and formatting it as "match_log_<counter>.csv".
 * It then writes the header line to the file and resets the
 * statistics for this run.
 */
void DataLogger::startLogging() {
    isLogging = true;

    // Generate a unique filename for this run
    char filebuf[50];
    sprintf(filebuf, "match_log_%d.csv", (int)getNextFile());
    filename = std::string(filebuf);

    writeHeader();

    // Reset statistics
    maxLeftTemp = maxRightTemp = 0;
    avgLeftPower = avgRightPower = 0;
    minVoltage = 999;
    count = 0;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Logging to %s", filename.c_str());
}

/**
 * Stops the current data logging session.
 *
 * This function stops the current data logging session and displays
 * the total number of points logged to the brain screen.
 */
void DataLogger::stopLogging() {
    isLogging = false;
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Logging stopped. Total points: %d", count);
}

/**
 * Logs a single data point to the SD card.
 *
 * This function logs a single data point to the SD card. The data point
 * contains information about the current state of the robot, including
 * motor velocities, temperatures, battery voltage and current, odometry
 * coordinates, and pneumatics usage. The data point is formatted as a CSV
 * line, with the same order as the header line written by writeHeader.
 * If the SD card is not currently logging, the function does nothing.
 *
 * @param driveBase The SixWheelDrive object containing motor information.
 * @param odom The OdometryTracker object containing odometry information.
 * @param pn The PneumaticsTracker object containing pneumatics usage information.
 * @param state The RobotState object containing the current state of the robot.
 * @param currEvent The current event of the robot (preauton, auton, driver).
 * @param joy3 The joystick input value for the third axis.
 * @param joy1 The joystick input value for the first axis.
 * @param optGrey The value of the optical grey sensor.
 */
void DataLogger::logData(SixWheelDrive& driveBase, OdometryTracker& odom, PneumaticsTracker& pn, RobotState& state, CurrEvent currEvent, double joy3, double joy1, double optGrey) {
    if (!isLogging) return;

    DataPoint point;
    point.timestamp = Brain.Timer.time(msec) + state.trueTimeCorr;
    point.currEvent = currEvent;
    point.joyStickInput3 = joy3;
    point.joyStickInput1 = joy1;
    point.leftMotorPower = driveBase.getLeftVelocity();
    point.rightMotorPower = driveBase.getRightVelocity();
    point.leftMotorTemp = driveBase.getLeftTemp();
    point.rightMotorTemp = driveBase.getRightTemp();
    point.batteryVoltage = Brain.Battery.voltage(volt);
    point.batteryCurrent = Brain.Battery.current(amp);
    point.posX = odom.getX();
    point.posY = odom.getY();
    point.heading = odom.getHeading();
    point.pneumaticUses = pn.currentUses;
    point.optGrey = optGrey;

    // Update stats incrementally
    if (point.leftMotorTemp > maxLeftTemp) maxLeftTemp = point.leftMotorTemp;
    if (point.rightMotorTemp > maxRightTemp) maxRightTemp = point.rightMotorTemp;
    if (point.batteryVoltage < minVoltage) minVoltage = point.batteryVoltage;
    avgLeftPower = ((avgLeftPower * count) + point.leftMotorPower) / (count + 1);
    avgRightPower = ((avgRightPower * count) + point.rightMotorPower) / (count + 1);
    count++;

    // Write immediately to SD using uint8_t buffer
    appendData(point);
}

/**
 * Displays match statistics to the brain screen.
 *
 * This function displays the maximum left motor temperature, maximum right motor
 * temperature, average left motor power, average right motor power, and minimum
 * battery voltage recorded during the match. The statistics are displayed
 * in a formatted table on the brain screen.
 *
 * @note If no data points have been logged (i.e. count == 0), the function does nothing.
 */
void DataLogger::displayStats() {
    if (count == 0) return;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Match Statistics:");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Max L Temp: %.1fC", maxLeftTemp);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Max R Temp: %.1fC", maxRightTemp);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Avg L Power: %.1f%%", avgLeftPower);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Avg R Power: %.1f%%", avgRightPower);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Min Voltage: %.2fV", minVoltage);
}

/**
 * Returns the filename of the current data logging session.
 *
 * This function returns the filename of the current data logging session as a
 * std::string. The filename is in the format "match_log_<counter>.csv",
 * where <counter> is a incrementing counter representing the number of data
 * logging sessions.
 *
 * @return The filename of the current data logging session as a std::string.
 */
std::string DataLogger::getFilename() const {
    return filename;
}

