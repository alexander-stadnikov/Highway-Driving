#pragma once

#include "Cartesian2D.h"
#include "Frenet2D.h"

#include "json.hpp"

#include <unordered_map>
#include <memory>

namespace udacity
{
    class Route;
    class Telemetry;

    class SensorFusion
    {
    public:
        explicit SensorFusion(const nlohmann::json &,
                              const std::shared_ptr<Route> &,
                              const std::shared_ptr<Telemetry> &);

    private:
        struct Vehicle
        {
            int id;
            Cartesian2D cartesian;
            Cartesian2D v;
            Frenet2D frenet;
        };

        std::unordered_map<int, Vehicle> m_vehicles;
    };
}