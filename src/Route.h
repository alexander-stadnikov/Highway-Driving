#pragma once

#include <string>
#include <vector>

struct Waypoint
{
    double x;
    double y;
    double s;
    double dx;
    double dy;

    explicit Waypoint(const std::string &data);
};

struct Route
{
    std::vector<Waypoint> waypoints;

    explicit Route(const std::string &csv);
};