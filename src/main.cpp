#include "boat_simulator.hpp"
#include "gps_sensor.hpp"
#include <fstream>
#include <iostream>

int main() {
    const double dt = 0.1;
    const int steps = 600;

    BoatState initial;
    initial.x = 0.0;
    initial.y = 0.0;
    initial.heading = 0.0; // east
    initial.speed = 5.0; //m/s

    BoatSimulator boat(initial, 0.05); //0.05 rad/s durig the turn
    GpsSensor gps(3.0, 42); //3 m noise, fixed seed

    std::ofstream out("output/run.csv");
    out << "t,true_x,true_y,meas_x,meas_y\n";

    for (int i = 0; i < steps; i++) {
        boat.step(dt);
        const BoatState& truth = boat.state();
        GpsMeasurement z = gps.measure(truth);

        out << boat.time() << ","
            << truth.x << "," << truth.y << ","
            << z.x << "," << z.y << "\n";
    }

    std::cout << "Wrote " << steps << " steps to output/run.csv\n";
    return 0;
}