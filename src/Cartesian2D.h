#pragma once

namespace udacity
{
    struct Cartesian2D
    {
        double x{0.0};
        double y{0.0};

        Cartesian2D() {}

        Cartesian2D(double x, double y)
            : x(x),
              y(y)
        {
        }
    };
}