#include <algorithm>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Constants to calculate "next_x" and "next_y".
size_t MAX_POLY_FIT_NUM = 10;
size_t NEXT_POINTS_NUM = 50;
int NEXT_X_INC = 1.0;

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // Transform waypoints to vehicle coordinate system. Take at most |MAX_POLY_FIT_NUM| points.
          // - Transit by (-px, -py)
          // - Rotate by -psi
          size_t n_pts = std::min(ptsx.size(), MAX_POLY_FIT_NUM);
          Eigen::VectorXd trans_ptsx(n_pts);
          Eigen::VectorXd trans_ptsy(n_pts);
          for (int i=0; i<n_pts; ++i) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            trans_ptsx(i) = dx*cos(-psi) - dy*sin(-psi);
            trans_ptsy(i) = dx*sin(-psi) + dy*cos(-psi);
          }

          /**
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          // Fit reference line with 2nd order poly line.
          VectorXd coeffs = polyfit(trans_ptsx, trans_ptsy, 3);

          double cte = polyeval(coeffs, 0);  // x=0
          double epsi = -atan(coeffs[1]);  // 1-order differentiation with x=0

          // x, y, psi, v, cte, epsi, steer and throttle.
          Eigen::VectorXd state(6 + 2);
          state << 0, 0, 0, v, cte, epsi, steer_value, throttle_value;
          //std::cout << "state: " << state << std::endl;


          vector<double> result = mpc.Solve(state, coeffs);
          steer_value = result[0];
          throttle_value = result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * Add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */
          for (int i=2; i<result.size(); i+=2) {
            mpc_x_vals.push_back(result[i]);
            mpc_y_vals.push_back(result[i+1]);
            //std::cout << "mpc_x: " << ans[i] << ", mpc_y: " << ans[i+1] << std::endl;
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * Add (x,y) points to list here, points are in reference to 
           * the vehicle's coordinate system the points in the simulator are 
           * connected by a Yellow line
           */
          for (int i=0; i<NEXT_POINTS_NUM; ++i) {
            double x = NEXT_X_INC * i; 
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(coeffs, x));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
