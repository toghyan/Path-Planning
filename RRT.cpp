#include "RRT.h"

RRT::RRT()
{
}

RRT::RRT(Car start_car, Car goal_car, double size_of_world, double radius_goal, double delta_time){
    start = start_car;
    goal = goal_car;
    world_size = size_of_world;
    r_goal = radius_goal;
    dt = delta_time;
}

RRT::~RRT()
{
    
}

*Car RRT::Random_Car(){
    
}

*Car RRT::find_nearest(Car end){
    
}

bool RRT::expand_tree(){
    
}
