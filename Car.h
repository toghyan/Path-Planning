#ifndef CAR_H
#define CAR_H

#include <iostream>
#include <string>

using namespace std;

class Car
{
public:
	// Constructor with no input. Sets all the member variables equal to zero.
	Car();
	// Constructor with the yaw angle input only.
	Car(double new_yaw);
	// Constructo with x and y coordinates and the yaw angle.
	Car(double new_x, double new_y, double new_yaw);
	// Deconstructor
	~Car();
	// Returns x coordinate
	double get_x() const;
	// Returns y coordinate
	double get_y() const;
	// Returns yaw angle
	double get_yaw() const;
	// Equality comparison
	friend bool operator ==(Car const & C1, Car const & C2);
	// Inequality comparison 
	friend bool operator !=(Car const & C1, Car const & C2);
	// cout operator (print)
	friend ostream& operator <<(ostream& out, const Car & C);

private:
	// Memeber variables
	double x, y, yaw;
	
};

#endif // CAR_H
