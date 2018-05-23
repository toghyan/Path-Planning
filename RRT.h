#ifndef RRT_H
#define RRT_H

#define _USE_MATH_DEFINES

#include "Car.h"
#include "PathPlanner.h"
#include <random>
#include <math.h>

using namespace std;

class RRT
{
public:
    RRT();
    // Constructor. Using start position of the car and defining the end goal as a circle. The size of the world is also
    // Part of the constructor. It defines the limit on x and y coordinate as -size_of_world =< x,y =< size_of_world
    RRT(Car start_car, Car goal_car, double size_of_world, double radius_goal, double delta_time);
    // Deconstructor
    ~RRT();
    // Generates random Car postions in the boundaries of the configuration space
    Car * Random_Car();
    // Finds the neerest Car in the tree to the input position
    Car * find_nearest(Car end);
    // Expands the tree and checks if the added node is in the goal zone
    bool expand_tree();
    
    
    
private:
    double world_size, dt, r_goal;
    Car start, goal; 
    
    

};

#endif // RRT_H
