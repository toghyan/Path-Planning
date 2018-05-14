#include "Car.h"
	


Car::Car(){
	x = 0;
	y = 0;
	yaw = 0;
}

Car::Car(double new_yaw){
	x = 0;
	y = 0;
	yaw = new_yaw;
}

Car::Car(double new_x, double new_y, double new_yaw){
	x = new_x;
	y = new_y;
	yaw = new_yaw;
}

Car::~Car(){
}

double Car::get_x() const{
	return x;
}

double Car::get_y() const{
	return y;
}

double Car::get_yaw() const{
	return yaw;
}

bool operator ==(Car const & C1, Car const & C2){
	return C1.x == C2.x && C1.y == C2.y && C1.yaw == C2.yaw;
}

bool operator !=(Car const & C1, Car const & C2){
	return !(C1==C2);
}

ostream& operator <<(ostream& out, const Car & C){
	out << "x = " << C.x << ", y = " << C.y << ", yaw = " << C.yaw;
	return out;
}

