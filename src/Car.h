#pragma once

#include <vector>
#include <memory>

#include "json.hpp"

#include "helpers.h"
#include "Telemetry.h"
#include "spline.h"

namespace udacity
{
    class Car
    {
    public:
        enum Lane
        {
            Left = 2,
            Middle = 6,
            Right = 10
        };
        struct PlannedPath
        {
            std::vector<double> x;
            std::vector<double> y;
        };

    public:
        Car(Lane);

        void update(const std::shared_ptr<Telemetry> &);
        Path path() const;

        void setRoute(const std::shared_ptr<Route> &);

    private:
        struct CarPosition
        {
            Cartesian2D pos;
            double yaw;
        };

        void addIntermediatePoints(const std::vector<double> &points,
                                   std::vector<double> &x, std::vector<double> &y) const;
        std::shared_ptr<tk::spline> createSpline(const CarPosition &) const;
        void convertFromGlobalToLocal(const CarPosition &, std::vector<double> &x,
                                      std::vector<double> &y) const;
        Path interpolatePath(const CarPosition &, const std::shared_ptr<tk::spline> &, int pathLength) const;

    private:
        Lane m_lane;
        std::shared_ptr<Telemetry> m_telemetry;
        std::shared_ptr<Route> m_route;
    };
}