#include "vehicle.h"


Vehicle::Vehicle()
{
}

Vehicle::Vehicle(double s_, double d_, double v_)
{
    update(s_, d_, v_);
}

void Vehicle::update(double s_, double d_, double v_) {
    s = s_;
    d = d_;
    v = v_;
    lane = Utils::d2lane(d);
}
