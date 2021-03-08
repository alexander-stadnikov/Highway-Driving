#include "Route.h"

#include <sstream>
#include <fstream>

Waypoint::Waypoint(const std::string &data)
{
    std::istringstream iss(data);
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;
}

Route::Route(const std::string &csv)
{
    std::ifstream in(csv.c_str(), std::ifstream::in);
    std::string line;

    while (getline(in, line))
    {
        waypoints.push_back(Waypoint(line));
    }
}