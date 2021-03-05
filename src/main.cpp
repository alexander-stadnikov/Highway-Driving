#include <uWS/uWS.h>

#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

void processMessage(uWS::WebSocket<uWS::SERVER>, char *, size_t, uWS::OpCode, const Map &map);

int main()
{
    uWS::Hub h;
    Map map("../data/highway_map.csv");
    double max_s = 6945.554;

    h.onMessage([&map](uWS::WebSocket<uWS::SERVER> ws, char *data,
                       size_t length, uWS::OpCode opCode) {
        processMessage(ws, data, length, opCode, map);
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

void processMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode, const Map &)
{
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
        auto s = hasData(data);

        if (!s.empty())
        {
            auto j = nlohmann::json::parse(s);
            std::string event = j[0].get<std::string>();

            if (event == "telemetry")
            {
                CarState carState(j);
                Path previousPath(j);

                // Sensor Fusion Data, a list of all other cars on the same side
                //   of the road.
                auto sensor_fusion = j[1]["sensor_fusion"];

                nlohmann::json msgJson;

                std::vector<double> next_x_vals;
                std::vector<double> next_y_vals;

                /**
                    * TODO: define a path made up of (x,y) points that the car will visit
                    *   sequentially every .02 seconds
                    */

                msgJson["next_x"] = next_x_vals;
                msgJson["next_y"] = next_y_vals;

                auto msg = "42[\"control\"," + msgJson.dump() + "]";

                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
        else // Manual driving
        {
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
    }
}