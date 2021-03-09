#pragma once

#include "Cartesian2D.h"
#include "Frenet2D.h"
#include "Path.h"

namespace udacity
{
    struct Telemetry
    {
        Cartesian2D cartesian;
        Frenet2D frenet;
        double yaw{0.0};
        double speed{0.0};
        Path previousPath;

        Telemetry() {}
    };
}