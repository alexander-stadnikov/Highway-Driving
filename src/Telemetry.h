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
        double yaw;
        double speed;
        Path previousPath;
    };
}