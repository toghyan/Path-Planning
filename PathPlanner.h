#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <math.h>
#include "Car.h"
#include "Circle.h"

class PathPlanner
{
public:
    PathPlanner();
    PathPlanner(Car const & c_start, Car const & c_end);
    ~PathPlanner();
    

};

#endif // PATHPLANNER_H
