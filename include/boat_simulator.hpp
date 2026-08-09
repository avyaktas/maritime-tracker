// Interface

#pragma once
#include "types.hpp"

class BoatSimulator {
public:
    BoatSimulator(BoatState initial, double turn_rate);

    //Advance true state forward by dt seconds
    void step(double dt);

    const BoatState& state() const { return state_; }
    double time() const { return t_; }

private: 
    BoatState state_;
    double turn_rate_;  //rad/s during turn phase
    double t_ = 0.0;    //elapsed sim time
};