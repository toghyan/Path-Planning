#include "PathPlanner.h"

PathPlanner::PathPlanner(){

}

PathPlanner::PathPlanner(Car const & c_start, Car const & c_end, double min_turn_r, double max_velocity){
    car_s = c_start;
    car_e = c_end;
    r_min = min_turn_r;
    vel = max_velocity;
}

PathPlanner::~PathPlanner(){
    
}

vector<double> PathPlanner::find_optimal_path(string & commands){
    vector<double> path(3);
    return path;
}

vector<double> PathPlanner::RSR(){
    vector<double> path(3);
    return path;
}

vector<double> PathPlanner::LSL(){
    vector<double> path(3);
    return path;
}

vector<double> PathPlanner::LSR(){
    vector<double> path(3);
    return path;
}

vector<double> PathPlanner::RLS(){
    vector<double> path(3);
    return path;
}

vector<double> PathPlanner::RLR(){
    vector<double> path(3);
    return path;
}

vector<double> PathPlanner::LRL(){
    vector<double> path(3);
    return path;
}

