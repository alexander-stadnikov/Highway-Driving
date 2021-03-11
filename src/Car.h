#pragma once

#include <vector>
#include <memory>

#include "json.hpp"

#include "helpers.h"
#include "Telemetry.h"
#include "spline.h"
#include "FSM.h"

namespace udacity
{
    class Car
    {
    public:
        using Trajectory = std::tuple<std::vector<double>, std::vector<double>>;

    public:
        Car();

        void update(const std::shared_ptr<Telemetry> &);
        Trajectory path() const;

        void setRoute(const std::shared_ptr<Route> &);

    private:
        struct CarPosition
        {
            Cartesian2D pos;
            double yaw;
        };

    private:
        void addIntermediatePoints(const std::vector<double> &points,
                                   std::vector<double> &x, std::vector<double> &y) const;
        std::shared_ptr<tk::spline> createSpline(const CarPosition &) const;
        void convertFromGlobalToLocal(const CarPosition &, std::vector<double> &x,
                                      std::vector<double> &y) const;
        Car::Trajectory interpolatePath(const CarPosition &,
                                        const std::shared_ptr<tk::spline> &, int pathLength) const;

    private:
        std::shared_ptr<Telemetry> m_telemetry;
        std::shared_ptr<Route> m_route;
        FSM m_fsm;
    };
}