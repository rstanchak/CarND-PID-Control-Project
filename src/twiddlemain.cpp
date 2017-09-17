#include "uws-compat.h"
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "TwiddleOptimizer.h"
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

int main(int argc, char ** argv)
{
  uWS::Hub h;

  struct Context {
	  PID pid;
	  TwiddleOptimizer<3> twiddle;
	  double average_cte2;
	  double cte2_sum;
	  double best_param[3];
	  int count;
	  bool connected;
	  int maxiter;
  } ctx;

  // read parameters from command line Kp, Ki, Kp, and N (number of simualation iterations)
  ctx.best_param[0] = strtod(argv[1], NULL);
  ctx.best_param[1] = strtod(argv[2], NULL);
  ctx.best_param[2] = strtod(argv[3], NULL);
  ctx.maxiter = strtol(argv[4], NULL, 10);

  for(int i=0; i<3; ++i) {
	  ctx.twiddle.p_[i] = ctx.best_param[i];
	  ctx.twiddle.best_[i] = ctx.best_param[i];
  }
  // Step size is 3% of best parameter
  ctx.twiddle.dp_[0] = ctx.best_param[0]*.03;
  ctx.twiddle.dp_[1] = ctx.best_param[1]*.03;
  ctx.twiddle.dp_[2] = ctx.best_param[2]*.03;

  ctx.average_cte2 = 0;
  ctx.cte2_sum = 0;
  ctx.count = 0;
  ctx.connected = false;

  // Initialize the pid variable.
  ctx.pid.Init(ctx.best_param[0], ctx.best_param[1], ctx.best_param[2]);

  h.onMessage( UWS_ON_MESSAGE([&ctx](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		  if(!ctx.connected) return;
	//std::cout<<data<<std::endl;
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          //double speed = std::stod(j[1]["speed"].get<std::string>()); 
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
		  ctx.cte2_sum += (cte*cte);
		  ++ctx.count;
		  ctx.average_cte2 = ctx.cte2_sum/ctx.count;

		  if(ctx.count >= ctx.maxiter){

			  // reset
			  std::string msg = "42[\"reset\",{}]";
			  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			  ctx.connected = false;
			  return;
		  }
          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
		  ctx.pid.UpdateError(cte);
		  steer_value = ctx.pid.CalculateOutput(); // * (M_1_PI);

		  if(steer_value < -1) steer_value = -1;
		  else if(steer_value > 1) steer_value = 1;
          
          // DEBUG
          //std::cout << "CTE: " << cte <<","<< ctx.average_cte2 << " Steering Value: " << steer_value << std::endl;
		  
		  // throttle is inversely proportional to steering value
		  double throttle = 0.60 * (1. - fabs(steer_value)) + 0.05;
		  //double throttle = 1.0;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  }));

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

  h.onConnection(UWS_ON_CONNECTION([&h,&ctx](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		  ctx.connected = true;
		  if(ctx.count > 0) {
			  // update optimizer, PID parameters
			  std::array<double, 3> last_params, best_params, next_params;
			  ctx.twiddle.get(last_params);
			  ctx.twiddle.update( ctx.cte2_sum/ctx.count );
			  ctx.twiddle.get(next_params);
			  while( next_params[0] < 0 || next_params[1] < 0 || next_params[2] < 0) {
				  ctx.twiddle.update( 10000L );
				  ctx.twiddle.get(next_params);
			  }
			  double best_error = ctx.twiddle.getBest(best_params);
			  std::cout << "LAST err=" << ctx.average_cte2 
				  << " p="<<last_params[0]<<","<<last_params[1]<<","<<last_params[2]
				  << "\tBEST err="<<best_error<<" p = "<<best_params[0]<<" "<<best_params[1]<<" "<<best_params[2]
				  << "\tNEXT p=["<<next_params[0]<<","<<next_params[1]<<","<<next_params[2]<<"]"<<std::endl;
			  ctx.average_cte2 = 0;
			  ctx.cte2_sum = 0;
			  ctx.count = 0;
			  ctx.pid.Init(next_params[0], next_params[1], next_params[2]);
		  }
  }));

  h.onDisconnection(UWS_ON_DISCONNECTION([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  }));

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
