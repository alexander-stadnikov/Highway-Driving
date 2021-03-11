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
        Route() noexcept;
        explicit Route(const std::string &csv) noexcept;
        Route(const Route &) noexcept;

        Route &operator=(const Route &) noexcept;

        Cartesian2D toCartesian(const Frenet2D &) const noexcept;

        void setMaxSpeedMph(const double maxMph) noexcept;
        double maxSpeed() const noexcept;
        double recommendedSpeed() const noexcept;

        void setNumberOfLanes(int numberOfLanes) noexcept;
        int frenetToLaneNumber(double d) const noexcept;
        double laneCenterToFrenet(int) const noexcept;

    private:
        struct Waypoint
        {
            double x{0.0};
            double y{0.0};
            double s{0.0};
            double dx{0.0};
            double dy{0.0};

            Waypoint() noexcept;
            explicit Waypoint(const std::string &) noexcept;

            static bool lessByS(const Waypoint &, const Waypoint &) noexcept;
        };

        std::vector<Waypoint> m_waypoints;
        double m_maxSpeed{0.0};
        int m_numberOfLanes{0};
    };
}