#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <math.h>
#include "Car.h"
#include "Circle.h"
#include <vector>
#include <string>

class PathPlanner
{
public:
    PathPlanner();
    PathPlanner(Car const & c_start, Car const & c_end, double mim_turn_r = 1, double max_velocity = 1);
    ~PathPlanner();
    // Modifies the command string to the appropriate command and returns the duration of each command
    double * find_optimal_path(string & commands);
    // Returns the length of the RSR path
    double RSR();
    // Returns the length of the LSL path
    double LSL();
    // Returns the length of the LSR path
    double LSR();
    // Returns the length of the RLS path
    double RLS();
    // Returns the length of the RLR path
    double RLR();
    // Returns the length of the LRL path
    double LRL();
    

};

#endif // PATHPLANNER_H
