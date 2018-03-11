#include "path_planner.h"



PathPlanner::PathPlanner()
{
    // initialize current state
    max_s = 6945.554;

    // start in middle lane
    lane = 1;

    // cold start
    ref_v = 0;
}

/**
 *
 */
void PathPlanner::updateTrajectory(
            VehicleState vState,
            vector<double> previous_path_x, vector<double> previous_path_y,
            vector<double> &next_path_x, vector<double> &next_path_y,
            double end_path_s, double end_path_d,
            const vector<vector<double>> &sensor_fusion)
{



    // current other car states and trajectories - sensor fusion

    // current ego state and multiple possible trajectories

    // evaluate trajectories with costs

    // update FSM state

            //////////////////////////////////////////////////////////////////////////////
            // get size of the path from preivous path
            int prev_size = previous_path_x.size();

            // utilize sensor fusion data to prevent collision
            if (prev_size > 0) {
                car_s = end_path_s;
            }
            bool too_close = false;

            // find ref_v to use
            for (int i = 0; i < sensor_fusion.size(); i++)
            {
                // car is in my lane
                float d = sensor_fusion[i][6];
                if (d < (2+4*lane+2) && d > (2+4*lane-2))
                {
                    // (id,x,y,vx,vy,s,d)
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];

                    double check_speed = sqrt(vx*vx+vy*vy);
                    double check_car_s = sensor_fusion[i][5];

                    // use previous points to project s value outward
                    check_car_s += ((double)prev_size * .02 * check_speed);
                    //
                    if ((check_car_s > car_s) && ((check_car_s-car_s) < 30))
                    {
                        too_close = true;
                    }
                }
            }

            if (too_close)
            {
                ref_vel -= .224; // 5 meter/sec
            }
            else if (ref_vel < 49.5)
            {
                ref_vel += .224;
            }

            // Prediction

            // Behavior

            buildTrajectory(vState,
                    previous_path_x, previous_path_y,
                    end_path_s, end_path_d,
                    next_path_x, next_path_y);

}

/**
 *
 */
void PathPlanner::buildTrajectory(const VehicleState &vState,
        const vector<double> p_x, const vector<double> p_y,
        vector<double> &t_x, vector<double> &t_y,
        double end_path_s, double end_path_d)
{
    // Create a list of widely spread (x,y) waypoints, evenly spaced at 30m
    // later we will interpolate these waypoints iwth a spline and fill it
    // with more points that control speed.
    vector<double> ptsx, ptsy;

    // reference x, y, yaw
    // previous paths end point or starting point
    double ref_x   = vState.x;
    double ref_y   = vState.y;
    double ref_yaw = Utils::deg2rad(vState.yaw);

    int prev_size = p_x.size();

    // if previous size is almost empty, use the car as starting reference
    if (prev_size < 2)
    {
        double car_x_prev = ref_x - cos(ref_yaw);
        double car_y_prev = ref_y - sin(ref_yaw);

        ptsx.push_back(car_x_prev);
        ptsy.push_back(car_y_prev);

        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y);
    }
    // use the previous path's end point as starting reference
    else {
        ref_x = p_x[prev_size-1];
        ref_y = p_y[prev_size-1];

        double ref_x_prev = p_x[prev_size-2];
        double ref_y_prev = p_y[prev_size-2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsy.push_back(ref_y_prev);

        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y);
    }

    // add evenly 30m spaced points ahead of the starting reference
    // road width: 4
    int car_s = (prev_size > 0) ? end_path_s : vState.s;

    for (int delta = 30; delta < 100; delta += 30)
    {
        vector<double> next_wp = wp.getXY(car_s + delta, (2+4*lane));
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }

    // rotate car reference angle to 0 degree
    for (int i = 0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
    }

    // build trajectory
    // Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

    // - previous predicted path
    for (int i = 0; i < prev_size; i++)
    {
        t_x.push_back(p_x[i]);
        t_y.push_back(p_y[i]);
    }

    // - new predicted path
    tk::spline s;
    s.set_points(ptsx, ptsy);

    // Calculate how to break up spline points so that we travel at our desired speed
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with previous points, we will always output 50 points.
    for (int i = 1; i <= 50 - prev_size; i++)
    {
        // N * .02 * vel = dist
        double N = (target_dist / (.02*ref_v/2.24)); // mile/hour -> meter/sec
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        t_x.push_back(x_point);
        t_y.push_back(y_point);
    }

}
