#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "utils.h"

using namespace std;

class Vehicle
{
    public:
        Vehicle();
        Vehicle(double s, double d, double v);
        void update(double s, double d, double v);

    private:
        int lane;
        int s, d, v;
};

#endif // VEHICLE_H
