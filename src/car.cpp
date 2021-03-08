#include "car.h"
#include "path.h"
#include "helpers.h"

using namespace udacity;

Car::Car(const Route &route, Lane lane, double ref_v, int max_path)
    : m_route(route),
      m_lane(lane),
      m_ref_v(ref_v),
      m_max_path(max_path)
{
}

Car::PlannedPath Car::path()
{
    PlannedPath path;
    double dist_inc = 0.5;
    for (int i = 0; i < m_max_path; ++i)
    {
        const auto next_s = m_s + (i + 1) * dist_inc;
        const auto next_xy = m_route.getXY(next_s, m_lane);
        path.x.push_back(next_xy[0]);
        path.y.push_back(next_xy[1]);
    }

    return path;
}

void Car::update(const nlohmann::json &json)
{
    setPosition(json);
    setSpeed(json);
}

void Car::setPosition(const nlohmann::json &json)
{
    m_x = json[1]["x"];
    m_y = json[1]["y"];
    m_s = json[1]["s"];
    m_d = json[1]["d"];
    m_yaw = json[1]["yaw"];

    Path path(json);
    m_prev_path_size = path.x.size();
}

void Car::setSpeed(const nlohmann::json &json)
{
    m_speed = json[1]["speed"];
}