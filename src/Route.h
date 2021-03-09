#pragma once

#include <string>
#include <vector>

#include "Cartesian2D.h"
#include "Frenet2D.h"

namespace udacity
{
    class Route final
    {
    public:
        Route();
        explicit Route(const std::string &csv);
        Route(const Route &);

        Route &operator=(const Route &);

        Cartesian2D toCartesian(const Frenet2D &) const;
        void setMaxSpeedMph(const double maxMph);
        double maxSpeed() const;

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
        double m_maxSpeed{0.0};
    };
}