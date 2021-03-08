#pragma once

#include <vector>

#include "json.hpp"

#include "helpers.h"

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

    Car(Lane, double ref_v, int max_path);

    void update(const nlohmann::json &json);
    PlannedPath path();

    void setRoute(const udacity::Route &);

private:
    void setPosition(const nlohmann::json &json);
    void setSpeed(const nlohmann::json &json);

    Lane m_lane;
    double m_ref_v;
    double m_x;
    double m_y;
    double m_s;
    double m_d;
    double m_yaw;
    double m_speed;
    int m_prev_path_size{0};
    const int m_max_path;
    udacity::Route m_route;
};