#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H


struct VehicleState
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;

    int lane;
};

#endif // VEHICLE_STATE_H
