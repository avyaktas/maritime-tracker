#include "boat_simulator.hpp"
#include "gps_sensor.hpp"
#include "kalman_filter.hpp"
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
    KalmanFilter kf(0.5, 3.0); //process noise, measirment noise

    std::ofstream out("output/run.csv");
    out << "t,true_x,true_y,meas_x,meas_y,est_x,est_y,est_vx,est_vy\n";

    for (int i = 0; i < steps; i++) {
        boat.step(dt);
        const BoatState& truth = boat.state();
        GpsMeasurement z = gps.measure(truth);

        kf.predict(dt);
        kf.update(z);
        const Eigen::Vector4d& x = kf.state();

        out << boat.time() << ","
            << truth.x << "," << truth.y << ","
            << z.x << "," << z.y << ","
            << x(0) << "," << x(1) << ","
            << x(2) << "," << x(3) << "\n";
    }

    std::cout << "Wrote " << steps << " steps to output/run.csv\n";
    return 0;
}