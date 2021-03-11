#include "Car.h"
#include "Route.h"
#include "helpers.h"

using namespace udacity;

Car::Car()
    : m_fsm(m_route)
{
}

Car::Trajectory Car::path() const
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
    m_fsm.update(m_telemetry);
}

void Car::setRoute(const std::shared_ptr<Route> &route)
{
    m_route = route;
}

void Car::addIntermediatePoints(const std::vector<double> &points,
                                std::vector<double> &x, std::vector<double> &y) const
{
    for (const auto p : points)
    {
        const auto wp = m_route->toCartesian({m_telemetry->frenet.s + p,
                                              m_route->laneCenterToFrenet(m_fsm.lane())});
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

Car::Trajectory Car::interpolatePath(const CarPosition &carPosition,
                                     const std::shared_ptr<tk::spline> &spline,
                                     int pathLength) const
{
    const double targetX = 30.0;
    const double targetY = (*spline)(targetX);
    const double dst = std::sqrt(targetX * targetX + targetY * targetY);
    const double N = dst / (0.02 * m_fsm.speed());
    const double dx = targetX / N;
    const double sinYaw = std::sin(carPosition.yaw);
    const double cosYaw = std::cos(carPosition.yaw);
    const int previousPathLength = m_telemetry->previousPath.waypoints.size();

    std::vector<double> trajectoryX(pathLength);
    std::vector<double> trajectoryY(pathLength);

    for (int i = 0; i < previousPathLength; i++)
    {
        trajectoryX[i] = m_telemetry->previousPath.waypoints[i].x;
        trajectoryY[i] = m_telemetry->previousPath.waypoints[i].y;
    }

    double x = 0.0;
    for (int i = previousPathLength; i < pathLength; i++)
    {
        x += dx;
        double y = (*spline)(x);

        trajectoryX[i] = carPosition.pos.x + x * cosYaw - y * sinYaw;
        trajectoryY[i] = carPosition.pos.y + x * sinYaw + y * cosYaw;
    }

    return {trajectoryX, trajectoryY};
}