#include "car.h"
#include "helpers.h"

Car::Car(const nlohmann::json &json, const Map &map, double ref_v)
    : m_x(json[1]["x"]),
      m_y(json[1]["y"]),
      m_s(json[1]["s"]),
      m_d(json[1]["d"]),
      m_yaw(json[1]["yaw"]),
      m_speed(json[1]["speed"]),
      m_map(map),
      m_ref_v(ref_v)
{
}

Car::PlannedPath Car::path()
{
    PlannedPath path;
    double dist_inc = 0.5;
    for (int i = 0; i < 50; ++i)
    {
        const auto next_s = m_s + (i + 1) * dist_inc;
        const auto next_xy = getXY(next_s, m_lane, m_map);
        path.x.push_back(next_xy[0]);
        path.y.push_back(next_xy[1]);
    }

    return path;
}