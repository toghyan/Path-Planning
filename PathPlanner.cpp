#include "PathPlanner.h"

PathPlanner::PathPlanner(){

}

PathPlanner::PathPlanner(Car const & c_start, Car const & c_end, double min_turn_r, double max_velocity){
    car_start = c_start;
    car_end = c_end;
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
    // Start (right) circle center coordinates 
    double x_center_start = car_start.get_x() + r_min * sin(car_start.get_yaw());
    double y_center_start = car_start.get_y() - r_min * cos(car_start.get_yaw());
    // End (right) circle center coordinates
    double x_center_end = car_end.get_x() + r_min * sin(car_end.get_yaw());
    double y_center_end = car_end.get_y() - r_min * cos(car_end.get_yaw());
    // Angle between the line connecting the centers and the positive x direction
    double alpha = atan2(y_center_end - y_center_start, x_center_end - x_center_start);

    // Angle of car at the start circle
    double theta_start = atan2(car_start.get_y() - y_center_start, car_start.get_x() - x_center_start);

    // Angle of car on end circle
    double theta_end = atan2(car_end.get_y() - y_center_end, car_end.get_x() - x_center_end);
    // Amount of rotation on the start circle 
    double Rotation_C1 = theta_start - alpha - M_PI_2;
    if(Rotation_C1 < 0){
        Rotation_C1 += 2 * M_PI;
    } 
    else if(Rotation_C1 > 2 * M_PI) {
        Rotation_C1 -= 2 * M_PI;
    }
    // Amount of rotation on the end circle
    double Rotation_C2 = alpha + M_PI_2 - theta_end;
    if(Rotation_C2 < 0){
        Rotation_C2 += 2 * M_PI;
    }
    else if(Rotation_C2 > 2 * M_PI) {
        Rotation_C2 -= 2 * M_PI;
    }
 
    path[0] = Rotation_C1 * r_min;
    path[1] = sqrt(pow(x_center_end - x_center_start,2) + pow(y_center_end - y_center_start,2));
    path[2] = Rotation_C2 * r_min;
    
    return path;
}

vector<double> PathPlanner::LSL(){
    vector<double> path(3);
    // Start (left) circle center coordinates 
    double x_center_start = car_start.get_x() - r_min * sin(car_start.get_yaw());
    double y_center_start = car_start.get_y() + r_min * cos(car_start.get_yaw());
    // End (left) circle center coordinates
    double x_center_end = car_end.get_x() - r_min * sin(car_end.get_yaw());
    double y_center_end = car_end.get_y() + r_min * cos(car_end.get_yaw());
    // Angle between the line connecting the centers and the positive x direction
    double alpha = atan2(y_center_end - y_center_start, x_center_end - x_center_start);
    
    // Angle of car at the start circle
    double theta_start = atan2(car_start.get_y() - y_center_start, car_start.get_x() - x_center_start);

    // Angle of car on end circle
    double theta_end = atan2(car_end.get_y() - y_center_end, car_end.get_x() - x_center_end);

    // Amount of rotation on the start circle 
    double Rotation_C1 = alpha - M_PI_2 - theta_start;
    while(Rotation_C1 < 0){
        Rotation_C1 += 2 * M_PI;
    } 
    while(Rotation_C1 > 2 * M_PI) {
        Rotation_C1 -= 2 * M_PI;
    }
    // Amount of rotation on the end circle
    double Rotation_C2 = theta_end - alpha + M_PI_2;
    while(Rotation_C2 < 0){
        Rotation_C2 += 2 * M_PI;
    }
    while(Rotation_C2 > 2 * M_PI) {
        Rotation_C2 -= 2 * M_PI;
    }
    
    path[0] = Rotation_C1 * r_min;
    path[1] = sqrt(pow(x_center_end - x_center_start,2) + pow(y_center_end - y_center_start,2));
    path[2] = Rotation_C2 * r_min;
    
    return path;
}

vector<double> PathPlanner::LSR(){
    vector<double> path(3);
    // Start (left) circle center coordinates 
    double x_center_start = car_start.get_x() - r_min * sin(car_start.get_yaw());
    double y_center_start = car_start.get_y() + r_min * cos(car_start.get_yaw());
    
    // End (right) circle center coordinates
    double x_center_end = car_end.get_x() + r_min * sin(car_end.get_yaw());
    double y_center_end = car_end.get_y() - r_min * cos(car_end.get_yaw());
    // center to center length
    double cen_to_cen = sqrt(pow(x_center_end - x_center_start,2) + pow(y_center_end - y_center_start,2));
    // Making sure the circles are not entangled
    if(cen_to_cen > 2 * r_min){
        throw("There is no valid RLS path!");
    }
    // angle between the line connecting the centers and the positive x direction
    double alpha = atan2(y_center_end - y_center_start, x_center_end - x_center_start);
    // Angle between the line conecting the centers and the tangent point
    double beta = acos(2 * r_min / cen_to_cen);
    // Angle of car at the start circle
    double theta_start = atan2(car_start.get_y() - y_center_start, car_start.get_x() - x_center_start);
    // Angle of car on end circle
    double theta_end = atan2(car_end.get_y() - y_center_end, car_end.get_x() - x_center_end);
    // Amount of rotation on the start circle
    double Rotation_C1 = alpha - beta - theta_start;
    while(Rotation_C1 < 0){
        Rotation_C1 += 2 * M_PI;
    } 
    while(Rotation_C1 > 2 * M_PI) {
        Rotation_C1 -= 2 * M_PI;
    }
    // Amount of rotation on the end circle
    double Rotation_C2 = alpha + M_PI - beta - theta_end;
    while(Rotation_C2 < 0){
        Rotation_C2 += 2 * M_PI;
    }
    while(Rotation_C2 > 2 * M_PI) {
        Rotation_C2 -= 2 * M_PI;
    }
    
    path[0] = Rotation_C1 * r_min;
    path[1] = sqrt(pow(cen_to_cen,2) - 4 * pow(r_min,2));
    path[2] = Rotation_C2 * r_min;
    
    return path;
}

vector<double> PathPlanner::RLS(){
    // Initialize the vector to be returned
    vector<double> path(3);
    // Start (right) circle center coordinates 
    double x_center_start = car_start.get_x() + r_min * sin(car_start.get_yaw());
    double y_center_start = car_start.get_y() - r_min * cos(car_start.get_yaw());
    // End (left) circle center coordinates
    double x_center_end = car_end.get_x() - r_min * sin(car_end.get_yaw());
    double y_center_end = car_end.get_y() + r_min * cos(car_end.get_yaw());
    // center to center length
    double cen_to_cen = sqrt(pow(x_center_end - x_center_start,2) + pow(y_center_end - y_center_start,2));
    // Making sure the circles are not entangled
    if(cen_to_cen > 2 * r_min){
        throw("There is no valid RLS path!");
    }
    // angle between the line connecting the centers and the positive x direction
    double alpha = atan2(y_center_end - y_center_start, x_center_end - x_center_start);
    // Angle between the line conecting the centers and the tangent point
    double beta = acos(2 * r_min / cen_to_cen);
    // Angle of car at the start circle
    double theta_start = atan2(car_start.get_y() - y_center_start, car_start.get_x() - x_center_start);
    // Angle of car on end circle
    double theta_end = atan2(car_end.get_y() - y_center_end, car_end.get_x() - x_center_end);
    // Amount of rotation on the start circle
    double Rotation_C1 = theta_start - alpha - beta;
    while(Rotation_C1 < 0){
        Rotation_C1 += 2 * M_PI;
    } 
    while(Rotation_C1 > 2 * M_PI) {
        Rotation_C1 -= 2 * M_PI;
    }
    // Amount of rotation on the end circle
    double Rotation_C2 = theta_end - alpha - beta - M_PI;
    while(Rotation_C2 < 0){
        Rotation_C2 += 2 * M_PI;
    }
    while(Rotation_C2 > 2 * M_PI) {
        Rotation_C2 -= 2 * M_PI;
    }
    
    path[0] = Rotation_C1 * r_min;
    path[1] = sqrt(pow(cen_to_cen,2) - 4 * pow(r_min,2));
    path[2] = Rotation_C2 * r_min;
    
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

