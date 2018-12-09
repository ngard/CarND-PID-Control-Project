#include <uWS/uWS.h>
#include <iostream>
#include <iomanip>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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
  return "";
}

int main()
{
  uWS::Hub h;

  bool do_twiddle = false;

  PID pid_steer;
  // TODO: Initialize the pid variable.
  double Kp_steer = 0.86561, Ki_steer = 0.005656, Kd_steer = 38.923;
  double dKp_steer = 0.1, dKi_steer = 0.001, dKd_steer = 3.;
  pid_steer.Init(Kp_steer, Ki_steer, Kd_steer, PID::KtoTune::Initial, PID::Trial::First);

  h.onMessage([&do_twiddle,&pid_steer,&Kp_steer,&Ki_steer,&Kd_steer,&dKp_steer,&dKi_steer,&dKd_steer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    static double best_error;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

	  // twiddle hyperparameters tuning
	  if (do_twiddle && pid_steer.TimeCount() > 2e4) {
	    double error = pid_steer.TotalError();
	    std::cerr << std::endl << "error:" << error << " best error:" << best_error << std::endl;
	    if (pid_steer.k == PID::KtoTune::Initial)
	      best_error = error;

	    switch(pid_steer.k) {
	    case PID::KtoTune::Initial:
	      Kp_steer += dKp_steer;
	      pid_steer.Init(Kp_steer,Ki_steer,Kd_steer,PID::KtoTune::TuneP,PID::Trial::First);
	      break;
	    case PID::KtoTune::TuneP:
	      if (pid_steer.trial == PID::Trial::First) {
		if (error < best_error) {
		  best_error = error;
		  dKp_steer *= 1.1;
		} else {
		  Kp_steer -= 2*dKp_steer;
		  pid_steer.Init(Kp_steer,Ki_steer,Kd_steer,PID::KtoTune::TuneP,PID::Trial::Second);
		  break;
		}
	      } else if (pid_steer.trial == PID::Trial::Second) {
		if (error < best_error) {
		  best_error = error;
		  dKp_steer *= 1.1;
		} else {
		  Kp_steer += dKp_steer;
		  dKp_steer *= 0.9;
		}
	      }
	      Ki_steer += dKi_steer;
	      pid_steer.Init(Kp_steer,Ki_steer,Kd_steer,PID::KtoTune::TuneI,PID::Trial::First);
	      break;
	    case PID::KtoTune::TuneI:
	      if (pid_steer.trial == PID::Trial::First) {
		if (error < best_error) {
		  best_error = error;
		  dKi_steer *= 1.1;
		} else {
		  Ki_steer -= 2*dKi_steer;
		  pid_steer.Init(Kp_steer,Ki_steer,Kd_steer,PID::KtoTune::TuneI,PID::Trial::Second);
		  break;
		}
	      } else if (pid_steer.trial == PID::Trial::Second) {
		if (error < best_error) {
		  best_error = error;
		  dKi_steer *= 1.1;
		} else {
		  Ki_steer += dKi_steer;
		  dKi_steer *= 0.9;
		}
	      }
	      Kd_steer += dKd_steer;
	      pid_steer.Init(Kp_steer,Ki_steer,Kd_steer,PID::KtoTune::TuneD,PID::Trial::First);
	      break;
	    case PID::KtoTune::TuneD:
	      if (pid_steer.trial == PID::Trial::First) {
		if (error < best_error) {
		  best_error = error;
		  dKd_steer *= 1.1;
		} else {
		  Kd_steer -= 2*dKd_steer;
		  pid_steer.Init(Kp_steer,Ki_steer,Kd_steer,PID::KtoTune::TuneD,PID::Trial::Second);
		  break;
		}
	      } else if (pid_steer.trial == PID::Trial::Second) {
		if (error < best_error) {
		  best_error = error;
		  dKd_steer *= 1.1;
		} else {
		  Kd_steer += dKd_steer;
		  dKd_steer *= 0.9;
		}
	      }
	      Kp_steer += dKp_steer;
	      pid_steer.Init(Kp_steer,Ki_steer,Kd_steer,PID::KtoTune::TuneP,PID::Trial::First);
	      break;
	    }
	  }

	  steer_value = pid_steer.CalcOutput(cte);
	  throttle = 0.45;

	  pid_steer.UpdateError(cte);

          // DEBUG
          // std::cerr << std::showpoint << std::showpos << std::fixed
	  // 	    << "CTE: " << cte
	  // 	    << " Steering Value: " << steer_value
	  // 	    << " Throttle: " << throttle
	  // 	    << " Average CTE: " << pid_steer.TotalError() << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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
