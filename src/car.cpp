#include "car.h"
#include "helpers.h"

Car::Car(const nlohmann::json &json)
    : m_x(json[1]["x"]),
      m_y(json[1]["y"]),
      m_s(json[1]["s"]),
      m_d(json[1]["d"]),
      m_yaw(json[1]["yaw"]),
      m_speed(json[1]["speed"])
{
}

Car::PlannedPath Car::path()
{
    PlannedPath path;
    double dist_inc = 0.5;
    for (int i = 0; i < 50; ++i)
    {
        const auto dp = (dist_inc * i);
        path.x.push_back(m_x + dp * cos(deg2rad(m_yaw)));
        path.y.push_back(m_y + dp * sin(deg2rad(m_yaw)));
    }

    return path;
}