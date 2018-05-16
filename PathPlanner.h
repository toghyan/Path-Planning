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
    vector<double> find_optimal_path(string & commands);
    // Returns the length of the RSR path
    vector<double> RSR(double const x_center_start, double const y_center_start, double const x_center_end, double const y_center_end);
    // Returns the length of the LSL path
    vector<double> LSL(double const x_center_start, double const y_center_start, double const x_center_end, double const y_center_end);
    // Returns the length of the LSR path
    vector<double> LSR(double const x_center_start, double const y_center_start, double const x_center_end, double const y_center_end);
    // Returns the length of the RLS path
    vector<double> RSL(double const x_center_start, double const y_center_start, double const x_center_end, double const y_center_end);
    // Returns the length of the RLR path
    vector<double> RLR(double const x_center_start, double const y_center_start, double const x_center_end, double const y_center_end);
    // Returns the length of the LRL path
    vector<double> LRL(double const x_center_start, double const y_center_start, double const x_center_end, double const y_center_end);
    
private: 
    Car car_start, car_end;
    double r_min, vel;
    

};

#endif // PATHPLANNER_H
