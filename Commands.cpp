#include "Commands.h"

Commands::Commands(string new_commands, double first_length, double second_length, double third_length)
{
    commands = new_commands;
    first = first_length;
    second = second_length;
    third = third_length;
}

Commands::~Commands()
{
}

double Commands::get_first() const{
    
    return first;
}

double Commands::get_second() const{
    
    return second;
}

double Commands::get_third() const{
    
    return third;
}

string Commands::get_commands() const{
    
    return commands;
}

bool operator ==(Commands const & C1, Commands const & C2){
    
    return C1.first == C2.first && C1.second == C2.second && C1.third == C2.third && C1.commands == C2.commands;
}

bool operator!=(Commands const & C1, Commands const & C2){
    
    return !(C1 == C2);
}

ostream& operator <<(ostream& out, Commands const & C){
    
    out << C.commands[0] << " : " << C.first << endl;
    out << C.commands[1] << " : " << C.second << endl;
    out << C.commands[2] << " : " << C.third << endl;
    
    return out;
}

