#include "gps_sensor.hpp"

GpsSensor::GpsSensor(double noise_std, unsigned int seed)
    : rng_(seed), noise_(0.0, noise_std) {} //constructs with mean 0, unbiasd GPS

GpsMeasurement GpsSensor::measure(const BoatState& truth) {
    GpsMeasurement z;
    z.x = truth.x + noise_(rng_);
    z.y = truth.y + noise_(rng_);
    return z;
}