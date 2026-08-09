#include "boat_simulator.hpp"
#include <cmath>


// constructor
BoatSimulator::BoatSimulator(BoatState initial, double turn_rate)
    : state_(initial), turn_rate_(turn_rate) {}

void BoatSimulator::step(double dt) {
    // phase 2 of trajectory, turn btw t=20s and t=40s
    if (t_ >= 20.0 && t_ < 40.0) {
        state_.heading += turn_rate_ *dt;
    }

    // move along curr heading
    state_.x += state_.speed * std::cos(state_.heading) * dt;
    state_.y += state_.speed * std::sin(state_.heading) * dt;

    t_ += dt;
}