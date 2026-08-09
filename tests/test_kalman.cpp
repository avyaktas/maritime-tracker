
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "kalman_filter.hpp"
#include "boat_simulator.hpp"
#include "gps_sensor.hpp"

#include <cmath>

TEST_CASE("Filter converges to truth with a noiseless sensor", "[kalman]") {
    BoatState initial;
    initial.speed = 5.0;
    initial.heading = 0.0;

    BoatSimulator boat(initial, 0.0);   // no turn: pure constant velocity
    GpsSensor perfect_gps(0.0, 1);      // zero noise
    KalmanFilter kf(0.1, 0.5);

    const double dt = 0.1;
    for (int i = 0; i < 200; ++i) {
        boat.step(dt);
        kf.predict(dt);
        kf.update(perfect_gps.measure(boat.state()));
    }

    const auto& x = kf.state();
    REQUIRE(std::abs(x(0) - boat.state().x) < 0.1);
    REQUIRE(std::abs(x(1) - boat.state().y) < 0.1);
    REQUIRE(std::abs(x(2) - 5.0) < 0.1);   // recovered vx it was never given
    REQUIRE(std::abs(x(3) - 0.0) < 0.1);
}

TEST_CASE("Uncertainty grows on predict and shrinks on update", "[kalman]") {
    KalmanFilter kf(0.5, 3.0);

    // Settle the filter first so P isn't dominated by the huge initial value.
    BoatState initial;
    initial.speed = 5.0;
    BoatSimulator boat(initial, 0.0);
    GpsSensor gps(3.0, 7);
    for (int i = 0; i < 100; ++i) {
        boat.step(0.1);
        kf.predict(0.1);
        kf.update(gps.measure(boat.state()));
    }

    const double p_settled = kf.covariance().trace();

    kf.predict(0.1);
    const double p_after_predict = kf.covariance().trace();
    REQUIRE(p_after_predict > p_settled);      // time passing = more doubt

    boat.step(0.1);
    kf.update(gps.measure(boat.state()));
    const double p_after_update = kf.covariance().trace();
    REQUIRE(p_after_update < p_after_predict); // observing = less doubt
}

TEST_CASE("Filter dead-reckons through GPS denial and recovers", "[kalman][gps-denied]") {
    BoatState initial;
    initial.speed = 5.0;
    BoatSimulator boat(initial, 0.0);
    GpsSensor gps(3.0, 99);
    KalmanFilter kf(0.5, 3.0);

    const double dt = 0.1;

    // Phase 1: normal operation, filter locks on.
    for (int i = 0; i < 200; ++i) {
        boat.step(dt);
        kf.predict(dt);
        kf.update(gps.measure(boat.state()));
    }
    const double err_before = std::hypot(kf.state()(0) - boat.state().x,
                                         kf.state()(1) - boat.state().y);

    // Phase 2: GPS denied for 5 seconds. Predict only — no measurements.
    for (int i = 0; i < 50; ++i) {
        boat.step(dt);
        kf.predict(dt);
    }
    const double err_denied = std::hypot(kf.state()(0) - boat.state().x,
                                         kf.state()(1) - boat.state().y);

    // It degrades, but stays usable — it knows its velocity.
    REQUIRE(err_denied > err_before);
    REQUIRE(err_denied < 10.0);

    // Phase 3: GPS returns. Filter should snap back.
    for (int i = 0; i < 100; ++i) {
        boat.step(dt);
        kf.predict(dt);
        kf.update(gps.measure(boat.state()));
    }
    const double err_after = std::hypot(kf.state()(0) - boat.state().x,
                                        kf.state()(1) - boat.state().y);
    REQUIRE(err_after < err_denied);
}
