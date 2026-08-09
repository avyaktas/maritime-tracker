#pragma once

// True state of the boat in the world.
// Only the simulator will see this, the filter won't

struct BoatState {
    double x = 0.0;     //pos East, meters
    double y = 0.0;     // pos North, meters
    double heading = 0.0;   //radians, 0 = pointing East
    double speed = 0.0;     //m/s
};

//What GPS reports: position only (noisy)
// no heading or speed for GPS measurement, 
// filter will recover velocity it was never given
struct GpsMeasurement {
    double x = 0.0;
    double y = 0.0;
};

