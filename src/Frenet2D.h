#pragma once

namespace udacity
{
    struct Frenet2D
    {
        double s{0.0};
        double d{0.0};

        Frenet2D() {}

        Frenet2D(double s, double d)
            : s(s),
              d(d)
        {
        }
    };
}