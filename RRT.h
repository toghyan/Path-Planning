#ifndef RRT_H
#define RRT_H

#include "Car.h"
#include "PathPlanner.h"

using namespace std;

class RRT
{
public:
    RRT();
    // Constructor. Using start position of the car and defining the end goal as a circle. The size of the world is also
    // Part of the constructor. It defines the limit on x and y coordinate as -size_of_world =< x,y =< size_of_world
    RRT(Car start, Car goal, double size_of_world, double radius_goal, double delta_time);
    // Deconstructor
    ~RRT();
    // Generates random Car postions in the boundaries of the configuration space
    Car Random_Car();
    // Finds the neerest Car in the tree to the input position
    *Car find_nearest(Car end);
    // Expands the tree
    void expand_tree();
    
    
    
Private:
    double world_size;
    

};

#endif // RRT_H
