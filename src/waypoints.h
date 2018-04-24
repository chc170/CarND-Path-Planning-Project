#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include "utils.h"
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

class Waypoints
{

    public:
        Waypoints();

        int getClosestWaypoint(double x , double y);
        int getNextWaypoint(double x, double y, double theta);

        vector<double> getFrenet(double x, double y, double theta);
        vector<double> getXY(double s, double d);

    private:
        vector<double> map_waypoints_x;
        vector<double> map_waypoints_y;
        vector<double> map_waypoints_s;
        vector<double> map_waypoints_dx;
        vector<double> map_waypoints_dy;
};

#endif // WAYPOINTS_H
