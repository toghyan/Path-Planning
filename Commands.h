/*
 * This class is used to store the commands of a dubins path. "commands" represents the type of the dubins path (RSR, LSL, etc) and
 * "first", "second", and "third" represent the length of each part of the path. 
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#include <string>
#include <iostream>

using namespace std;

class Commands
{
public:
    Commands(string new_commands, double first_length, double second_length, double third_length);
    ~Commands();
    double get_first() const;
    double get_second() const;
    double get_third() const;
    string get_commands() const;
    friend bool operator ==(Commands const & C1, Commands const & C2);
    friend bool operator!=(Commands const & C1, Commands const & C2);
    friend ostream& operator <<(ostream& out, Commands const & C); 
    
private:
    double first, second, third;
    string commands;

};

#endif // COMMANDS_H
