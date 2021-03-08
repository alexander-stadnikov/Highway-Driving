#include "Route.h"

#include <sstream>
#include <fstream>
#include <algorithm>
#include <cmath>

Route::Waypoint::Waypoint()
{
}

Route::Waypoint::Waypoint(const std::string &data)
{
    std::istringstream iss(data);
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;
}

bool Route::Waypoint::lessByS(const Waypoint &lhs, const Waypoint &rhs)
{
    return std::isless(lhs.s, rhs.s);
}

Route::Route(const std::string &csv)
{
    std::ifstream in(csv.c_str(), std::ifstream::in);
    std::string line;

    while (getline(in, line))
    {
        m_waypoints.push_back(Waypoint(line));
    }
}

std::vector<double> Route::getXY(double s, double d) const
{
    Waypoint w;
    w.s = s;
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
    const double segmentS = s - wp->s;
    const double segmentX = wp->x + segmentS * cos(heading);
    const double segmentY = wp->y + segmentS * sin(heading);
    const double orthogonalHeading = heading - M_PI / 2;
    const double x = segmentX + d * cos(orthogonalHeading);
    const double y = segmentY + d * sin(orthogonalHeading);

    return {x, y};
}