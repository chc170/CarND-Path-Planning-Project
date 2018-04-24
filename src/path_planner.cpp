#include "path_planner.h"


const double DELTA_V = .224; // 5 meter / sec
const double MAX_V = 50;     // mile / hr
const double MIN_V = 0;
const int DIST_THRESHOLD = 30;

using namespace std;

PathPlanner::PathPlanner()
{
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
            vector<double> prev_path_x, vector<double> prev_path_y,
            vector<double> &next_path_x, vector<double> &next_path_y,
            double end_path_s, double end_path_d,
            const vector<vector<double>> &sensor_fusion)
{
    int prev_size = prev_path_x.size();
    double car_s = end_path_s;

    // current ego state and multiple possible trajectories
    vector<double>* best_path_x;
    vector<double>* best_path_y;
    VehicleState* best_state;
    VehicleState* state;
    double best_v = ref_v;
    int best_cost = std::numeric_limits<int>::max();
    int cost;

    int start_l = max(0, lane-1);
    int end_l = min(2, lane+1);
    for (int l=start_l; l <= end_l; l++) {
        double start_v = max(MIN_V, ref_v-DELTA_V);
        double end_v = min(MAX_V, ref_v+DELTA_V);
        for (double v = start_v; v <= end_v; v += DELTA_V) {
            if (v <= 0) { continue; }
            cost = 0;
            // create possible state
            state = new VehicleState();
			state->x = vState.x;
            state->y = vState.y;
            state->s = vState.s;
            state->d = vState.d;
            state->yaw = vState.yaw;
            state->speed = v;
            state->lane = l;

            // create possible path
            vector<double> possible_path_x;
            vector<double> possible_path_y;
            buildTrajectory(*state,
                            prev_path_x, prev_path_y,
                            possible_path_x, possible_path_y,
                            end_path_s, end_path_d);

            // evaluate trajectory with other cars
            for (const auto& other : sensor_fusion) {
                // car is in my target lane
                float d = other[6];
                if (d < (2+4*l+2) && d > (2+4*l-2)) {
                    double vx = other[3];
                    double vy = other[4];

                    double cur_s = sqrt(vx*vx+vy*vy);
                    double proj_s = other[5] +
                                        ((double)prev_size * .02 * cur_s);
                    if ((proj_s > car_s) && ((proj_s-car_s) < DIST_THRESHOLD)) {
                        cost += 1000 * (proj_s-car_s);
                    }
                }
            }
            if (cost > 1000) {
                cost += v * 100;
            } else {
			    cost += (MAX_V - v) * 100;
            }
			if (l != lane) {cout << "change lane ";  cost += 50; }
			cout << "Option lane: " << l << " speed: " << v << " cost: " << cost << endl;
            if (cost < best_cost) {
				best_cost = cost;
                best_state = state;
            }
        }
    }
    cout << "Cost: " << best_cost << endl;
    cout << "REF V: " << ref_v << endl;
    cout << "START SPEED: " << max(MIN_V, ref_v-DELTA_V) << endl;
    cout << "END SPEED: " << min(MAX_V, ref_v+DELTA_V) << endl;
    cout << "SPEED: " << best_v << endl;
    cout << "LANE: " << best_state->lane << endl;

	ref_v = best_state->speed;
	lane = best_state->lane;
    buildTrajectory(*best_state,
					prev_path_x, prev_path_y,
                    next_path_x, next_path_y,
                    end_path_s, end_path_d);
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
        vector<double> next_wp = wp.getXY(car_s + delta, (2+4*vState.lane));
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
        double N = (target_dist / (.02*vState.speed/2.24)); // mile/hour -> meter/sec
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
