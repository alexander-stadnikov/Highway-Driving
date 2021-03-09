#pragma once

#include "Cartesian2D.h"

#include <vector>

namespace udacity
{
    struct Path
    {
        std::vector<Cartesian2D> waypoints;
    };
}