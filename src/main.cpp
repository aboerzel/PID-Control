#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

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

/**
 * \brief Helper method to restart the simulator
 * \param ws Websocket
 */
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
{
    // reset
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
    uWS::Hub h;

    // Construct and initialize PID controller
    PID pid;
    //steer_pid.init(0.2, 0.001, 5.0); // initial parameter
    //steer_pid.init(0.220103, 0.000993, 4.999348);  // twiddle with constant speed about 10 mph
    //steer_pid.init(0.22639450, 0.00117619, 5.06311788);  
  
    //pid.init(0.2, 0.01, 2.0, 0.05, 0.005, 0.5);
    pid.init(0.01021000, 0.00101000, 1.00070000, 0.1, 0.5, 0.05, true, false); // initial parameter tried out by constant speed of about 10 mph

    //Steer parameter Kp : 0.01109828 Ki : 0.00112153 Kd : 1.10077000)



    h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode)
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
                    pid.update_error(cte, speed, angle);
                    const auto steer_control = pid.steer_control();
                    const auto speed_control = pid.speed_control();

                    // DEBUG
                    std::cout << "CTE: " << cte << " Speed: " << speed << " Angle: " << angle << std::endl;
                    std::cout << " Steer ctrl: " << steer_control << " Speed ctrl: " << speed_control << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_control;
                    msgJson["throttle"] = 0.2; // speed_control;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    // optimize control coefficients using twiddle
                    if (pid.twiddle())
                    {
                        // restart simulator if twiddle has updated the coefficients
                        reset_simulator(ws);
                    }
                                              
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

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
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

