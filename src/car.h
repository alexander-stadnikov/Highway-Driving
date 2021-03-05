#pragma once

#include <vector>

#include "json.hpp"

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

    Car(const nlohmann::json &json);

    PlannedPath path();

private:
    const double m_x;
    const double m_y;
    const double m_s;
    const double m_d;
    const double m_yaw;
    const double m_speed;
    Lane m_lane{Middle};
};