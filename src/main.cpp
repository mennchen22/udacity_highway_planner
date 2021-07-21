#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <algorithm>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
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
    /** Constants */
    static double MAX_SPEED = 49.5;
    static double PLANNING_RANGE = 30.0;
    static int MAX_WAYPOINTS_PLANNED = 50;
    static double CAR_UPDATE_SPEED = 0.02;

    static double DEACCELERATION = 0.224;
    static double ACCELERATION = 0.445;

    static double MID_LANE_D_OFFSET = 2;
    static double LANE_WIDTH = 4;

    /** Variables */
    int current_lane = 1;

    double ref_vel = 0.0;

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &current_lane, &ref_vel,
                        &map_waypoints_dx, &map_waypoints_dy]
                        (uWS::WebSocket <uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (!s.empty()) {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    int prev_size = previous_path_x.size();

                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }

                    bool to_close = false;

                    // find ref vel to use
                    for (auto &fusion_item : sensor_fusion) {
                        // car in my lane
                        float d = fusion_item[6];

                        // If car is within our lane
                        if (d < (MID_LANE_D_OFFSET + LANE_WIDTH * current_lane + (LANE_WIDTH / 2)) &&
                            d > (MID_LANE_D_OFFSET + LANE_WIDTH * current_lane - (LANE_WIDTH / 2))) {
                            double vx = fusion_item[3];
                            double vy = fusion_item[4];
                            double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
                            double check_car_s = fusion_item[5];

                            check_car_s += ((double) prev_size * CAR_UPDATE_SPEED *
                                            check_speed); // if using prev points can project s values out
                            if ((check_car_s > car_s) && ((check_car_s - car_s) < PLANNING_RANGE)) {
                                // Do your stuff here, lower vel and do not crash
                                to_close = true;
                            }
                        }
                    }

                    if (to_close) {
                        ref_vel -= DEACCELERATION;
                    } else {
                        ref_vel += ACCELERATION;
                        ref_vel = ref_vel <= 0 ? 0 : ref_vel >= MAX_SPEED ? MAX_SPEED : ref_vel;
                    }


                    // create a list of widely spaced (x,y) waypoints, evenly spaces in a given range
                    // later we will interpolate these waypoints with a spline and fill it in with more points to control the car smoothly
                    vector<double> pts_x;
                    vector<double> pts_y;

                    // reference x, y, yaw states
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    // if prev size is almost empty, use the car as a starting reference
                    if (prev_size < 2) {
                        // Use two points that make the path tangent to the car
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        pts_x.push_back(prev_car_x);
                        pts_x.push_back(car_x);

                        pts_y.push_back(prev_car_y);
                        pts_y.push_back(car_y);
                    }
                        // use the previous path and points as start
                    else {
                        // Redefine reference data state as prev path and point
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_y - ref_x_prev);

                        // use the two points that makes the path tangent to the prev path's and points
                        pts_x.push_back(ref_x_prev);
                        pts_x.push_back(ref_x);

                        pts_y.push_back(ref_y_prev);
                        pts_y.push_back(ref_y);
                    }

                    // In Frenet add evenly spaced points ahead of the starting reference
                    vector<double> next_wp0 = getXY(car_s + PLANNING_RANGE,
                                                    (MID_LANE_D_OFFSET + LANE_WIDTH * current_lane),
                                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + PLANNING_RANGE * 2,
                                                    (MID_LANE_D_OFFSET + LANE_WIDTH * current_lane),
                                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + PLANNING_RANGE * 3,
                                                    (MID_LANE_D_OFFSET + LANE_WIDTH * current_lane),
                                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    pts_x.push_back(next_wp0[0]);
                    pts_x.push_back(next_wp1[0]);
                    pts_x.push_back(next_wp2[0]);

                    pts_y.push_back(next_wp0[1]);
                    pts_y.push_back(next_wp1[1]);
                    pts_y.push_back(next_wp2[1]);

                    // Transform to car centered coordinates
                    for (int i = 0; i < pts_x.size(); i++) {
                        double shift_x = pts_x[i] - ref_x;
                        double shift_y = pts_y[i] - ref_y;

                        pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                    }

                    // create spline
                    tk::spline spline;
                    spline.set_points(pts_x, pts_y);

                    // Create next points vectors
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    // Start with all prev points from last time
                    for (int i = 0; i < prev_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // Calculate how to break up the spline points so that we travel at our desired reference velocity
                    double target_x = PLANNING_RANGE;
                    double target_y = spline(target_x);
                    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

                    double x_add_on = 0.0;

                    // Fill the rest evenly to have x points in the list
                    for (int i = 0; i < MAX_WAYPOINTS_PLANNED - prev_size; i++) {

                        double N = (target_dist / (CAR_UPDATE_SPEED * ref_vel / 2.24));
                        double x_point = x_add_on + (target_x) / N;
                        double y_point = spline(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        // transform back to map coord
                        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }

                    std::cout << "Speed: " << ref_vel << " kmh" << std::endl;
                    if (current_lane == 1) {
                        if(to_close){
                            std::cout << "Lane: |   |   | 0 |   |   |   |"  << std::endl;
                        }else{
                            std::cout << "Lane: |   |   |   |   |   |   |" << std::endl;
                        }

                        std::cout << "Lane: |   |   |   |   |   |   |" << std::endl;
                        std::cout << "Lane: |   |   |   | X |   |   |"  << std::endl;
                    } else if (current_lane == 2) {
                        if(to_close){
                            std::cout << "Lane: |   |   |   | 0 |   |   |"  << std::endl;
                        }else{
                            std::cout << "Lane: |   |   |   |   |   |   |" << std::endl;
                        }

                        std::cout << "Lane: |   |   |   |   |   |   |" << std::endl;
                        std::cout << "Lane: |   |   |   |   | X |   |" << std::endl;
                    } else if (current_lane == 3) {
                        if(to_close){
                            std::cout << "Lane: |   |   |   |   |   | 0 |"  << std::endl;
                        }else{
                            std::cout << "Lane: |   |   |   |   |   |   |" << std::endl;
                        }

                        std::cout << "Lane: |   |   |   |   |   |   |"  << std::endl;
                        std::cout << "Lane: |   |   |   |   |   | X |" << std::endl;
                    }


                    // Create json update
                    json msgJson;

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket <uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket <uWS::SERVER> ws, int code,
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