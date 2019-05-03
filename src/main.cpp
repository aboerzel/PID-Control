#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

#define MIN_SPEED 10.0
#define MAX_SPEED 50.0

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    PID steering_controller(0.115, 0.0, 2.0);
    steering_controller.twiddle_one_param(2, 0.01, 0.000001, 100, 400);

    PID speed_controller(0.1, 0.0, 1.0);
    //speed_controller.twiddle_all_params(0.01, 0.0, 0.01, 0.000001, 100, 400);
   
    h.onMessage([&steering_controller, &speed_controller](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(string(data).substr(0, length));

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    const auto cte = std::stod(j[1]["cte"].get<string>());
                    const auto speed = std::stod(j[1]["speed"].get<string>());
                    const auto angle = std::stod(j[1]["steering_angle"].get<string>());

                    /**
                     * TODO: Calculate steering value here, remember the steering value is
                     *   [-1, 1].
                     * NOTE: Feel free to play around with the throttle and speed.
                     *   Maybe use another PID controller to control the speed!
                     */         

                    // update errors and get control values for steering and speed
                    steering_controller.update_error(cte);       
                    const auto steer_control = steering_controller.control();

                    // The smaller the steering angle, the greater the target speed
                    const auto target_speed = (MAX_SPEED - MIN_SPEED) * (1.0 - steer_control * steer_control) + MIN_SPEED;
                    const auto speed_error = speed - target_speed;

                    speed_controller.update_error(speed_error);
                    const auto speed_control = speed_controller.control();
                                                    
                    printf("**************** Control Monitoring ****************\n");
                    printf("Speed        : %3.4f\n", speed);
                    printf("Angle        : %3.4f\n", angle);
                    printf("CTE          : %3.4f\n", cte);
                    printf("Speed error  : %3.4f\n", speed_error);
                    printf("Target speed : %3.4f\n", target_speed);
                    printf("Steer ctrl   : %3.4f\n", steer_control);
                    printf("Speed ctrl   : %3.4f\n", speed_control);
                
                    json msgJson;
                    msgJson["steering_angle"] = steer_control;
                    msgJson["throttle"] = 0.3; // speed_control;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); 
       
                } // end "telemetry" if
            }
            else
            {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        } // end websocket message if
    }); // end h.onMessage

    h.onConnection([&h, &steering_controller, &speed_controller](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
        steering_controller.set_server(ws);
        speed_controller.set_server(ws);
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char* message, size_t length)
    {
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

