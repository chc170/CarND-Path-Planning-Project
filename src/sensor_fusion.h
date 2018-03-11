#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

using namespace std;

class SensorFusion
{
    public:
        SensorFusion();

        void update(const vector<vector<double>> &sensor_fusion);

        vector<vector<vector<double>>> getTrajectories();

    private:
        Waypoints wp;

        unordered_map<int, Vehicle> otherVehicles;
};

#endif // SENSOR_FUSION_H:w
