#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "utils.h"
#include "waypoints.h"
#include "vehicle_state.h"
#include "sensor_fusion.h"


class PathPlanner
{
    public:
        PathPlanner();

        void updateTrajectory(
            VehicleState vState,
            vector<double> previous_path_x, vector<double> previous_path_y,
            vector <double> &next_path_x, vector<double> &next_path_y,
            double end_path_s, double end_path_d,
            const vector<vector<double>> &sensor_fusion
        );

        void buildTrajectory(
            VehicleState vState,
            const vector<double> p_x, const vector<double> p_y,
            vector <double> &t_x, vector<double> &t_y,
            double end_path_s, double end_path_d,
        );

    private:
        SensorFusion sf;
        Waypoints wp;

        double max_s;
        double velocity;
        int lane;
};

#endif // PATH_PLANNER_H
