#ifndef VEHICLE_H
#define VEHICLE_H

#include "utils.h"


class Vehicle
{
    public:
        Vehicle();
        Vhiecle(double s, double d, double v);

        // get current state

        // get previous path

        vector<vector<double>> buildTrajectory();

    private:
        int lane;
        int s, d, v;
};

#endif // VEHICLE_H
