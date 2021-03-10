#pragma once

#include <vector>
#include <memory>
#include <unordered_map>

#include "json.hpp"

#include "helpers.h"
#include "Telemetry.h"
#include "spline.h"

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
        void setLane(size_t lane) noexcept;

    private:
        struct CarPosition
        {
            Cartesian2D pos;
            double yaw;
        };

        enum class State
        {
            Accelerate,
            Brake,
            KeepLane,
            ChangeLeft,
            ChangeRight
        };

        void addIntermediatePoints(const std::vector<double> &points,
                                   std::vector<double> &x, std::vector<double> &y) const;
        std::shared_ptr<tk::spline> createSpline(const CarPosition &) const;
        void convertFromGlobalToLocal(const CarPosition &, std::vector<double> &x,
                                      std::vector<double> &y) const;
        Car::Trajectory interpolatePath(const CarPosition &,
                                        const std::shared_ptr<tk::spline> &, int pathLength) const;

    private:
        size_t m_lane;
        std::shared_ptr<Telemetry> m_telemetry;
        std::shared_ptr<Route> m_route;
        State m_state;
        const std::unordered_map<State, std::vector<State>> m_fsm;
    };
}