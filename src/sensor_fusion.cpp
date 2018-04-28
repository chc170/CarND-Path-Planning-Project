#include "sensor_fusion.h"


SensorFusion::SensorFusion()
{}

void SensorFusion::update(const vector<vector<double>> &sensor_fusion)
{
    // initialize the flags
    unordered_map<int, bool> existingVehicles;
    for (auto &v : otherVehicles)
    {
        existingVehicles[v.first] = false;
    }

    // update vehicles
    for (auto &data : sensor_fusion)
    {
        int id    = data[0];
        double x  = data[1];
        double y  = data[2];
        double vx = data[3];
        double vy = data[4];
        double v  = sqrt(vx*vx+vy*vy);
        double s  = data[5];
        double d  = data[6];

        auto vehicle = otherVehicles.find(id);
        if (vehicle == otherVehicles.end())
        {
            otherVehicles[id] = Vehicle(s, d, v);
        }
        else
        {
            otherVehicles[id].update(s, d, v);
            existingVehicles[id] = true;
        }
    }

    // remove vehicles that are lost track
    for (auto &vehicle : existingVehicles)
    {
        if (!vehicle.second)
        {
            otherVehicles.erase(vehicle.first);
        }
    }
}
