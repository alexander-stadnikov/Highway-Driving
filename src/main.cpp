#include <uWS/uWS.h>

#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "helpers.h"
#include "Car.h"
#include "Route.h"
#include "Telemetry.h"
#include "SensorFusion.h"

void processMessage(uWS::WebSocket<uWS::SERVER>, char *, size_t, uWS::OpCode,
                    udacity::Car &ca, const std::shared_ptr<udacity::Route> &);
std::shared_ptr<udacity::Telemetry> createTelemetry(const nlohmann::json &);

int main()
{
    uWS::Hub h;
    auto route = std::make_shared<udacity::Route>("../data/highway_map.csv");
    route->setMaxSpeedMph(49.5);
    route->setNumberOfLanes(3);

    udacity::Car car;
    car.setRoute(route);

    h.onMessage([&car, &route](uWS::WebSocket<uWS::SERVER> ws, char *data,
                               size_t length, uWS::OpCode opCode) {
        processMessage(ws, data, length, opCode, car, route);
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

void processMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                    uWS::OpCode, udacity::Car &car, const std::shared_ptr<udacity::Route> &route)
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
                const auto tm = createTelemetry(j);
                const udacity::SensorFusion sensorFusion(j, route, tm);

                car.update(tm, sensorFusion);

                nlohmann::json outMsg;
                std::tie(outMsg["next_x"], outMsg["next_y"]) = car.path();
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

std::shared_ptr<udacity::Telemetry> createTelemetry(const nlohmann::json &j)
{
    auto tm = std::make_shared<udacity::Telemetry>();
    tm->cartesian = {j[1]["x"], j[1]["y"]};
    tm->frenet = {j[1]["s"], j[1]["d"]};
    tm->yaw = j[1]["yaw"];
    tm->speed = j[1]["speed"];

    std::vector<double> x = j[1]["previous_path_x"];
    std::vector<double> y = j[1]["previous_path_y"];

    tm->previousPath.waypoints.resize(x.size());
    for (size_t i = 0; i < x.size(); ++i)
    {
        tm->previousPath.waypoints[i].x = x[i];
        tm->previousPath.waypoints[i].y = y[i];
    }

    return tm;
}