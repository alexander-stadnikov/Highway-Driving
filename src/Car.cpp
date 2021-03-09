#include "Car.h"
#include "Route.h"
#include "helpers.h"

using namespace udacity;

Car::Car(Lane lane)
    : m_lane(lane)
{
}

Path Car::path() const
{
    const int previousPathSize = m_telemetry->previousPath.waypoints.size();
    CarPosition carPosition{{m_telemetry->cartesian.x, m_telemetry->cartesian.y},
                            m_telemetry->yaw};
    if (previousPathSize >= 2)
    {
        const auto &p1 = m_telemetry->previousPath.waypoints[previousPathSize - 1];
        const auto &p2 = m_telemetry->previousPath.waypoints[previousPathSize - 2];
        carPosition.pos.x = p1.x;
        carPosition.pos.y = p1.y;
        carPosition.yaw = std::atan2(p1.y - p2.y, p1.x - p2.x);
    }

    auto spline = createSpline(carPosition);
    return interpolatePath(carPosition, spline, 50);
}

void Car::update(const std::shared_ptr<Telemetry> &tm)
{
    m_telemetry = tm;
}

void Car::setRoute(const Route &route)
{
    m_route = route;
}

void Car::addIntermediatePoints(const std::vector<double> &points,
                                std::vector<double> &x, std::vector<double> &y) const
{
    for (const auto p : points)
    {
        const auto wp = m_route.toCartesian({m_telemetry->frenet.s + p,
                                             static_cast<double>(m_lane)});
        x.push_back(wp.x);
        y.push_back(wp.y);
    }
}

std::shared_ptr<tk::spline> Car::createSpline(const CarPosition &carPosition) const
{
    const int previousPathSize = m_telemetry->previousPath.waypoints.size();
    std::vector<double> x;
    std::vector<double> y;

    if (previousPathSize < 2)
    {
        x.push_back(carPosition.pos.x - std::cos(carPosition.yaw));
        x.push_back(carPosition.pos.x);
        y.push_back(carPosition.pos.y - std::sin(carPosition.yaw));
        y.push_back(carPosition.pos.y);
    }
    else
    {
        const auto &p1 = m_telemetry->previousPath.waypoints[previousPathSize - 1];
        const auto &p2 = m_telemetry->previousPath.waypoints[previousPathSize - 2];
        x.push_back(p2.x);
        y.push_back(p2.y);
        x.push_back(p1.x);
        y.push_back(p1.y);
    }

    addIntermediatePoints({30.0, 60.0, 90.0}, x, y);
    convertFromGlobalToLocal(carPosition, x, y);
    auto f = std::make_shared<tk::spline>();
    f->set_points(x, y);
    return f;
}

void Car::convertFromGlobalToLocal(const CarPosition &carPos, std::vector<double> &x,
                                   std::vector<double> &y) const
{
    const double sinYaw = std::sin(-carPos.yaw);
    const double cosYaw = std::cos(-carPos.yaw);
    for (int i = 0; i < x.size(); i++)
    {
        const double shiftX = x[i] - carPos.pos.x;
        const double shiftY = y[i] - carPos.pos.y;
        x[i] = shiftX * cosYaw - shiftY * sinYaw;
        y[i] = shiftX * sinYaw + shiftY * cosYaw;
    }
}

Path Car::interpolatePath(const CarPosition &carPosition, const std::shared_ptr<tk::spline> &spline,
                          int pathLength) const
{
    Path nextPath = m_telemetry->previousPath;

    const double targetX = 30.0;
    const double targetY = (*spline)(targetX);
    const double dst = std::sqrt(targetX * targetX + targetY * targetY);
    double xAddOn = 0.0;
    const double N = dst / (0.02 * m_route.maxSpeed());
    const double dx = targetX / N;
    const double sinYaw = std::sin(carPosition.yaw);
    const double cosYaw = std::cos(carPosition.yaw);
    const int previousPathSize = m_telemetry->previousPath.waypoints.size();

    for (int i = 1; i <= pathLength - previousPathSize; i++)
    {
        xAddOn += dx;

        double x = xAddOn;
        double y = (*spline)(x);
        const double X = x;
        const double Y = y;

        x = carPosition.pos.x + X * cosYaw - Y * sinYaw;
        y = carPosition.pos.y + X * sinYaw + Y * cosYaw;
        nextPath.waypoints.push_back({x, y});
    }

    return nextPath;
}