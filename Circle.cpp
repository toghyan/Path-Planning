#include "Circle.h"

Circle::Circle(){
    right = true;
    x_center = 0;
    y_center = 0;
    r_min = 1;
}	

Circle::Circle(Car new_car, double new_r_min, bool new_is_right){
    right = new_is_right;
    r_min = new_r_min;
    car = new_car;
    if(right){
        x_center = car.get_x() + r_min * sin(car.get_yaw());
        y_center = car.get_y() - r_min * cos(car.get_yaw());
    }
    else {
        x_center = car.get_x() - r_min * sin(car.get_yaw());
        y_center = car.get_y() + r_min * cos(car.get_yaw());
    }
}	
    
Circle::~Circle(){
    
}

double Circle::get_xc() const{
    return x_center;
}

double Circle::get_yc() const{
    return y_center;
}

double Circle::get_r_min() const{
    return r_min;
}

bool Circle::is_right() const{
    return right;
}

ostream& operator <<(ostream& out, const Circle & C){
    out << "Center : (" << C.x_center << "," << C.y_center << ") " << endl;
    out << "Minimum turning radius = " << C.r_min << endl;
    out << "This is a ";
    if(C.right){
        out << "right circle." << endl;
    }
    else{
        out << "left circle." << endl;
    }
	return out;
}
