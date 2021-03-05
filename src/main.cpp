#include <uWS/uWS.h>

#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "helpers.h"
#include "car.h"

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

void processMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode, const Map &map)
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
                Car car(j, map);
                Path previousPath(j);
                SensorFusion sensorFusion(j);

                nlohmann::json outMsg;
                const auto plannedPath = car.path();
                outMsg["next_x"] = plannedPath.x;
                outMsg["next_y"] = plannedPath.y;
                auto msg = "42[\"control\"," + outMsg.dump() + "]";

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