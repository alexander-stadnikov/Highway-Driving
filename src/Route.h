#pragma once

#include <string>
#include <vector>

namespace udacity
{
    class Route
    {
    public:
        explicit Route(const std::string &csv);

        std::vector<double> getXY(double s, double d) const;

    private:
        struct Waypoint
        {
            double x{0.0};
            double y{0.0};
            double s{0.0};
            double dx{0.0};
            double dy{0.0};

            Waypoint();
            explicit Waypoint(const std::string &);

            static bool lessByS(const Waypoint &, const Waypoint &);
        };

        std::vector<Waypoint> m_waypoints;
    };
}