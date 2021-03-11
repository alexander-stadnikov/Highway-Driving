#include "SensorFusion.h"

#include "Route.h"
#include "Telemetry.h"

#include <cmath>
#include <iostream>

namespace
{
    int laneInFront(int lane)
    {
        return 100 * lane;
    }
}

namespace udacity
{
    SensorFusion::SensorFusion(const nlohmann::json &json,
                               const std::shared_ptr<Route> &route,
                               const std::shared_ptr<Telemetry> &tm) noexcept
        : m_tm(tm)
    {
        for (const auto &j : json[1]["sensor_fusion"])
        {
            Vehicle v{j[0],
                      {j[1], j[2]},
                      {j[3], j[4]},
                      {j[5], j[6]}};

            const auto NewDistance = m_tm->frenet.s - v.frenet.s;
            auto lane = route->frenetToLaneNumber(v.frenet.d);

            if (lane < 0 || lane >= route->numberOfLanes())
            {
                continue;
            }

            if (std::isless(NewDistance, 0.0))
            {
                lane = laneInFront(lane);
            }

            if (m_vehicles.count(lane) == 0)
            {
                m_vehicles[lane] = v;
                continue;
            }

            const auto ExistingDistance = std::fabs(m_tm->frenet.s - m_vehicles[lane].frenet.s);
            if (std::isless(std::fabs(NewDistance), ExistingDistance))
            {
                m_vehicles[lane] = v;
            }
        }

        // for (const auto &it : m_vehicles)
        // {
        //     if (it.first >= 100)
        //     {
        //         std::cout << "FRONT: " << it.first / 100;
        //     }
        //     else
        //     {
        //         std::cout << "BACK: " << it.first;
        //     }
        //     std::cout << " -> " << it.second.id << " : " << std::fabs(tm->frenet.s - it.second.frenet.s) << std::endl;
        // }
        // std::cout << "-------------------------------------------" << std::endl;
    }

    double SensorFusion::freeDistanceInFront(int lane) const noexcept
    {
        lane = laneInFront(lane);
        if (m_vehicles.count(lane) == 0)
        {
            return Unlimited;
        }

        return m_vehicles.at(lane).frenet.s - m_tm->frenet.s;
    }

    double SensorFusion::speedOfVehicleInFront(int lane) const noexcept
    {
        lane = laneInFront(lane);
        if (m_vehicles.count(lane) == 0)
        {
            return Unlimited;
        }

        const auto &v = m_vehicles.at(lane).v;
        return std::sqrt(std::pow(v.x, 2) + std::pow(v.y, 2));
    }
}