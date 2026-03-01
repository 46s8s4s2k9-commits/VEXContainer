// AutonRoutes.hpp - Autonomous Route Definitions and Registry

#ifndef AUTONROUTES_HPP
#define AUTONROUTES_HPP

#include "AutonRoute.hpp"
#include <vector>

enum class ballPathCommand {
    High,
    Low,
    Mid,
    Store,
    Delay,
    None
};

extern ballPathCommand pathCmd;
extern vex::mutex pathCmdMutex;

void ballPathLoop();

///////////////////////////////////////////////////
// Route Implementation Forward Declarations
/////////////////////////////////////////////////////////////////////

void leftAuton();
void rightAuton();
void leftMatchLoader();
void rightMatchLoader();

void OdomleftBallAuton();


///////////////////////////////////////////////////
// Route Registry Function
/////////////////////////////////////////////////////////////////////

/**
 * @brief Get the complete list of all available autonomous routes
 * @return Vector of AutonRoute structures containing all route definitions
 * 
 * This function returns a vector of all autonomous routes available for selection.
 * Add new routes to the implementation of this function in AutonRoutes.cpp
 */
std::vector<AutonRoute> getAutonRoutes();

#endif // AUTONROUTES_HPP