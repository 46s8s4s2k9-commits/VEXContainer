// AutonRoute.hpp

#ifndef AUTONROUTE_HPP
#define AUTONROUTE_HPP

#include <string>
#include <functional>

/**
 * @struct AutonRoute
 * @brief Defines a single autonomous route with all its metadata and execution function
 * 
 * This structure centralizes all information about an autonomous route in one place,
 * making it easy to add new routes without modifying multiple code sections.
 */
struct AutonRoute {
    std::string name;           // Display name (e.g., "Left", "Right Ball")
    std::string shortName;      // Short name for compact display (e.g., "L", "RB")
    std::function<void()> execute;  // Function to execute this route
    
    /**
     * @brief Constructor for AutonRoute
     * @param name Display name for the route
     * @param shortName Short name for compact displays
     * @param execute Function pointer to the route's execution function
     */
    AutonRoute(const std::string& name, const std::string& shortName, std::function<void()> execute)
        : name(name), shortName(shortName), execute(execute) {}
};

#endif // AUTONROUTE_HPP