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

//vector<vector<double>> Vehicle::buildTrajectory()
//{
//    vector<vector<double>> t;
//
//    int steps = 50;
//    double delta = .02
//    double nxt_s = s;
//
//    for (int i = 1; i <= steps; i++)
//    {
//        nxt_s += v * delta * steps / 2.24;
//        t.push_back({nxt_s, d, v});
//    }
//
//    return t;
//}
