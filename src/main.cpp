#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "json.hpp"
#include "spline.h"

#include "vehicle_state.h"
#include "path_planner.h"


using namespace std;
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
    uWS::Hub h;

    PathPlanner path_planner;

    h.onMessage([&path_planner](
          uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    auto carData = j[1];

                    // Main car's localization Data
                    const VehicleState vState = {
                        carData["x"],   carData["y"],
                        carData["s"],   carData["d"],
                        carData["yaw"], carData["speed"]
                    };

                    // Previous path data given to the Planner
                    auto previous_path_x = carData["previous_path_x"];
                    auto previous_path_y = carData["previous_path_y"];

                    // Previous path's end s and d values
                    double end_path_s = carData["end_path_s"];
                    double end_path_d = carData["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = carData["sensor_fusion"];

                    // update current trajectory
                    vector<double> next_path_x;
                    vector<double> next_path_y;

                    path_planner.updateTrajectory(
                        vState,
                        previous_path_x, previous_path_y,
                        next_path_x, next_path_y,
                        end_path_s, end_path_d,
                        sensor_fusion
                    );

                    // send trajectory to simulator
                    json msgJson;
                    msgJson["next_x"] = next_path_x;
                    msgJson["next_y"] = next_path_y;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";
                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
        res->end(s.data(), s.length());
      } else {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
      ws.close();
      std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
      std::cout << "Listening to port " << port << std::endl;
    } else {
      std::cerr << "Failed to listen to port" << std::endl;
      return -1;
    }
    h.run();
}
