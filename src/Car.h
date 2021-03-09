#pragma once

#include <vector>

#include "json.hpp"

#include "helpers.h"
#include "Telemetry.h"

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

        Car(Lane, int max_path);

        void update(const Telemetry &);
        PlannedPath path();

        void setRoute(const Route &);

    private:
        Lane m_lane;
        Telemetry m_telemetry;
        int m_prev_path_size{0};
        const int m_max_path;
        Route m_route;
    };
}