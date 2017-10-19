#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

//using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
//  return "";
}

int main()
{
  uWS::Hub h;


  PID steerPid;
  PID throttlePid;


  // initialize the steering angle and throttole values.

    steerPid.Init(0.0975998,0.000364879,1.61652);
 // steerPid.Init(0.12,0.00048,2.36);
 // steerPid.Init(0.1233,0.0001,2.36);
 // steerPid.Init(0.241063,0.000409787,2.66072);
  throttlePid.Init(0.0968345,0.0000,0.38775);
    
    

  h.onMessage([&steerPid,&throttlePid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
//          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
            // initialize variables.
          double steer_value = 0.0;
          double throttle_value = 0.0;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // DEBUG

          // update Steering angle error function.
          steerPid.UpdateError(cte);
          // calculate PID controller for steering angle
          steer_value = steerPid.GetValue();
          // Normalization
          steer_value = std::min(std::max(steer_value, -1.0), 1.0); //normalize the steer_value in range [-1,1]
            
          // link steer_value to target speed, so any veering will leads to slower speed.
          double target_speed = 60.0*(1.-std::fabs(steer_value))+20;
          // calculate the delta with respect to target speed and update the error function
          throttlePid.UpdateError(speed - target_speed);
          // calcualte PID controller for throttle
          throttle_value = throttlePid.GetValue();
          // Normalization
          throttle_value = std::min(std::max(throttle_value, -1.0), 1.0); //normalize the throttle in range [-1,1]
            
          json msgJson;
            
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else
        {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
   }
  });
    
  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
        
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
