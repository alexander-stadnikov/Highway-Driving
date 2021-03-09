#include "Car.h"
#include "path.h"
#include "helpers.h"

using namespace udacity;

Car::Car(Lane lane, int max_path)
    : m_lane(lane),
      m_max_path(max_path)
{
}

Car::PlannedPath Car::path()
{
    PlannedPath path;
    double dist_inc = 0.5;
    for (int i = 0; i < m_max_path; ++i)
    {
        auto next_s = m_telemetry.frenet.s + (i + 1) * dist_inc;
        const auto next_xy = m_route.toCartesian({next_s, static_cast<double>(m_lane)});
        path.x.push_back(next_xy.x);
        path.y.push_back(next_xy.y);
    }

    return path;
}

void Car::update(const Telemetry &tm)
{
    m_telemetry = tm;
}

void Car::setRoute(const Route &route)
{
    m_route = route;
}