#include "Route.h"

#include <sstream>
#include <fstream>
#include <algorithm>
#include <cmath>

using namespace udacity;

Route::Waypoint::Waypoint() noexcept
{
}

Route::Waypoint::Waypoint(const std::string &data) noexcept
{
    std::istringstream iss(data);
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;
}

bool Route::Waypoint::lessByS(const Waypoint &lhs, const Waypoint &rhs) noexcept
{
    return std::isless(lhs.s, rhs.s);
}

Route::Route() noexcept
{
}

Route::Route(const std::string &csv) noexcept
{
    std::ifstream in(csv.c_str(), std::ifstream::in);
    std::string line;

    while (getline(in, line))
    {
        m_waypoints.push_back(Waypoint(line));
    }
}

Route::Route(const Route &other) noexcept
{
    *this = other;
}

Route &Route::operator=(const Route &other) noexcept
{
    m_waypoints = other.m_waypoints;
    m_maxSpeed = other.m_maxSpeed;
    return *this;
}

Cartesian2D Route::toCartesian(const Frenet2D &f) const noexcept
{
    Waypoint w;
    w.s = f.s;
    auto nextWp = std::upper_bound(m_waypoints.cbegin(), m_waypoints.cend(), w, Waypoint::lessByS);
    auto wp = m_waypoints.cend() - 1;
    if (nextWp == m_waypoints.cend())
    {
        nextWp = m_waypoints.cbegin();
    }
    else
    {
        wp = nextWp - 1;
    }

    const double heading = atan2(nextWp->y - wp->y, nextWp->x - wp->x);
    const double segmentS = f.s - wp->s;
    const double segmentX = wp->x + segmentS * cos(heading);
    const double segmentY = wp->y + segmentS * sin(heading);
    const double orthogonalHeading = heading - M_PI / 2;
    const double x = segmentX + f.d * cos(orthogonalHeading);
    const double y = segmentY + f.d * sin(orthogonalHeading);

    return {x, y};
}

double Route::maxSpeed() const noexcept
{
    return m_maxSpeed;
}

void Route::setMaxSpeedMph(const double maxMph) noexcept
{
    m_maxSpeed = maxMph / 2.24;
}

void Route::setNumberOfLanes(int numberOfLanes) noexcept
{
    m_numberOfLanes = numberOfLanes;
}

int Route::frenetToLaneNumber(double d) const noexcept
{
    return static_cast<int>(d) / 4 - (std::isless(d, 0.0) ? 1 : 0);
}

double Route::laneCenterToFrenet(int lane) const noexcept
{
    return static_cast<double>(lane * 4 + (lane < 0 ? -2 : 2));
}

double Route::recommendedSpeed() const noexcept
{
    return maxSpeed() - 0.5;
}

int Route::numberOfLanes() const noexcept
{
    return m_numberOfLanes;
}