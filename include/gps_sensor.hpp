#pragma once
#include "types.hpp"
#include <random>

class GpsSensor { 
public: 
    //noise_std: standard dev of position error, in meters
    //seed: fixed so every run produces identical noise
    GpsSensor(double noise_std, unsigned int seed);

    //Take true state, return a corrupted observation of it
    GpsMeasurement measure(const BoatState& truth);

private: 
    std::mt19937 rng_;  //Mersenne twister = PRNG
    std::normal_distribution<double> noise_;    //distribution
};