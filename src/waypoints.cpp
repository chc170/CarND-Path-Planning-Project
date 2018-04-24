#include "waypoints.h"

Waypoints::Waypoints()
{
    string map_file = "../data/highway_map.csv";
    ifstream in_map_(map_file.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x, y;
        float s, d_x, d_y;

        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;

        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
}

int Waypoints::getClosestWaypoint(double x, double y)
{
    double closestLen = 100000; // a larget number
    int closestWaypoint = 0;

    for (int i = 0; i < map_waypoints_x.size(); i++)
    {
        double map_x = map_waypoints_x[i];
        double map_y = map_waypoints_y[i];
        double dist = Utils::distance(x, y, map_x, map_y);

        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

int Waypoints::getNextWaypoint(double x, double y, double theta)
{
    int closestWaypoint = getClosestWaypoint(x, y);

    double map_x = map_waypoints_x[closestWaypoint];
    double map_y = map_waypoints_y[closestWaypoint];

    double heading = atan2((map_y-y), (map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2 * M_PI - angle, angle);

    if (angle > M_PI/4)
    {
        closestWaypoint++;
        if (closestWaypoint == map_waypoints_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

/**
 * Transform from Cartesian x, y coordinates to Frenet s, d coordinataes
 */
vector<double> Waypoints::getFrenet(double x, double y, double theta)
{
    int next_wp = getNextWaypoint(x, y, theta);
    int prev_wp = (next_wp > 0) ? next_wp - 1 : map_waypoints_x.size() - 1 ;

    double n_x = map_waypoints_x[next_wp] - map_waypoints_x[prev_wp];
    double n_y = map_waypoints_y[next_wp] - map_waypoints_y[prev_wp];

    double x_x = x - map_waypoints_x[prev_wp];
    double x_y = y - map_waypoints_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = Utils::distance(x_x, x_y, proj_x, proj_y);

    // set if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - map_waypoints_x[prev_wp];
    double center_y = 2000 - map_waypoints_y[prev_wp];
    double centerToPos = Utils::distance(center_x, center_y, x_x, x_y);
    double centerToRef = Utils::distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++)
    {
        frenet_s += Utils::distance(map_waypoints_x[i], map_waypoints_y[i],
                                    map_waypoints_x[i+1], map_waypoints_y[i+1]);
    }
    frenet_s += Utils::distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

/**
 * Transform from Frenet s, d coordinates to Cartesian x, y
 */
vector<double> Waypoints::getXY(double s, double d)
{
    int prev_wp = -1;

    while((s > map_waypoints_s[prev_wp+1]) &&
          (prev_wp < (int)(map_waypoints_s.size()-1)))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1) % map_waypoints_x.size();

    double heading = atan2(
        (map_waypoints_y[wp2] - map_waypoints_y[prev_wp]),
        (map_waypoints_x[wp2] - map_waypoints_x[prev_wp])
    );

    // the x,y,s along the segment
    double seg_s = (s - map_waypoints_s[prev_wp]);

    double seg_x = map_waypoints_x[prev_wp] + seg_s * cos(heading);
    double seg_y = map_waypoints_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading-M_PI/2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

