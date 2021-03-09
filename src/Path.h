#pragma once

#include "Cartesian2D.h"

#include <vector>
#include <tuple>
namespace udacity
{
    struct Path
    {
        std::vector<Cartesian2D> waypoints;
    };
}