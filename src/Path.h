#pragma once

#include "Cartesian2D.h"

#include <vector>
#include <tuple>
namespace udacity
{
    struct Path
    {
        std::vector<Cartesian2D> waypoints;

        std::tuple<std::vector<double>, std::vector<double>> decompose() const
        {
            std::vector<double> x(waypoints.size());
            std::vector<double> y(waypoints.size());

            for (size_t i = 0; i < waypoints.size(); i++)
            {
                x[i] = waypoints[i].x;
                y[i] = waypoints[i].y;
            }

            return std::make_tuple(x, y);
        }
    };
}