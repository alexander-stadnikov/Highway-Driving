#include "FSM.h"
#include "Route.h"
#include "Telemetry.h"
#include "SensorFusion.h"

#include <cmath>
#include <iostream>

namespace
{
    double accelerate(double speed)
    {
        return speed + 1.5 * 0.224;
    }

    double brake(double speed)
    {
        return speed - 1.5 * 0.224;
    }

    struct Behaviour
    {
        using State = udacity::FSM::State;

        size_t lane;
        double speed;
        State state;
        double cost;

        Behaviour(State s, const std::shared_ptr<udacity::Telemetry> &tm,
                  const std::shared_ptr<udacity::Route> &route,
                  const udacity::SensorFusion &sf)
        {
            const auto currentLane = route->frenetToLaneNumber(tm->frenet.d);
            state = s;

            switch (state)
            {
            case State::KeepLane:
                lane = currentLane;
                break;

            case State::ChangeLeft:
                lane = currentLane - 1;
                break;

            case State::ChangeRight:
                lane = currentLane + 1;
                break;
            }

            speed = expectedSpeed(route, sf);
            cost = calculateCost(tm, route, sf);
        }

    private:
        double expectedSpeed(const std::shared_ptr<udacity::Route> &route,
                             const udacity::SensorFusion &sf) const noexcept
        {
            const double dst = sf.freeDistanceInFront(lane);
            const double speedInFront = sf.speedOfVehicleInFront(lane);

            const double speedLimit = route->maxSpeed() - 0.2;
            const double safetyDst = std::min(safetyDistance(speedLimit),
                                              safetyDistance(speedInFront));
            double speed = 0.0;

            if (dst > safetyDst)
            {
                return speedLimit;
            }
            else if (dst > 2.0 * safetyDst / 3.0)
            {
                return speedInFront;
            }

            return speedInFront - 5.0;
        }

        static double safetyDistance(double speed) noexcept
        {
            const double safetyReactionTime = 1.5;
            return safetyReactionTime * speed;
        }

        double keepOrPreferRight(int currentLane) const noexcept
        {
            double k = 0.0;

            if (lane == currentLane)
            {
                k = -10.0;
            }
            else if (lane == currentLane + 1)
            {
                k = -11.0;
            }

            return 10e2 * k;
        }

        double stayOnTheRoad(int numberOfLanes) const noexcept
        {
            return lane < 0 || lane >= numberOfLanes
                       ? 10e6
                       : 0.0;
        }

        static double overtake(double dst, double lim, double speed) noexcept
        {
            if (std::isless(dst, 75) && speed != udacity::SensorFusion::Unlimited)
            {
                return 10 * (lim - speed) * 10e2;
            }

            return 0.0;
        }

        static double avoidCollision(double front, double back, double safety) noexcept
        {
            double cost = 0;
            if (front < 10)
            {
                cost += 10e6;
            }
            else if (front < safety * .25)
            {
                cost += 10e5;
            }

            if (back < 6.5)
            {
                cost += 10e6;
            }

            return cost;
        }

        static double takeFreeLane(double front) noexcept
        {
            return -1 * std::min(75.0, front) * 10e2;
        }

        double keepUntilFast(double currentSpeed,
                             int currentLane) const noexcept
        {
            return std::isless(currentSpeed, 25.0) && lane == currentLane
                       ? -5 * 10e6
                       : 0.0;
        }

        double calculateCost(const std::shared_ptr<udacity::Telemetry> &tm,
                             const std::shared_ptr<udacity::Route> &route,
                             const udacity::SensorFusion &sf) const noexcept
        {
            const double dstInFront = sf.freeDistanceInFront(lane);
            const double dstBehind = sf.freeDistanceBehind(lane);
            const double speedLimit = route->maxSpeed();
            double safetyDst = safetyDistance(route->maxSpeed());
            const auto currentLane = route->frenetToLaneNumber(tm->frenet.d);

            return keepOrPreferRight(currentLane) +
                   stayOnTheRoad(route->numberOfLanes()) +
                   overtake(dstInFront, speedLimit, sf.speedOfVehicleInFront(lane)) +
                   avoidCollision(dstInFront, dstBehind, safetyDst) +
                   takeFreeLane(dstInFront) +
                   keepUntilFast(tm->speed, currentLane);
        }
    };
}

namespace udacity
{
    FSM::FSM(const std::shared_ptr<Route> &route)
        : m_state(State::KeepLane),
          m_transitions({{State::KeepLane, {State::KeepLane, State::ChangeLeft, State::ChangeRight}},
                         {State::ChangeLeft, {State::KeepLane, State::ChangeLeft}},
                         {State::ChangeRight, {State::KeepLane, State::ChangeRight}}}),
          m_route(route),
          m_speed(0),
          m_lane(0)
    {
    }

    void FSM::update(const std::shared_ptr<udacity::Telemetry> &tm,
                     const SensorFusion &sensorFusion) noexcept
    {
        auto minCost = std::numeric_limits<double>::max();
        auto safeSpeed = tm->speed;

        for (const auto nextState : m_transitions.at(m_state))
        {
            Behaviour potentialBehaviour(nextState, tm, m_route, sensorFusion);

            if (potentialBehaviour.cost < minCost)
            {
                minCost = potentialBehaviour.cost;
                m_state = potentialBehaviour.state;
                safeSpeed = potentialBehaviour.speed;
            }
        }

        transit(tm, safeSpeed);
    }

    void FSM::transit(const std::shared_ptr<udacity::Telemetry> &tm,
                      double safeSpeed) noexcept
    {
        const auto currentLane = m_route->frenetToLaneNumber(tm->frenet.d);
        m_speed = std::isless(m_speed, safeSpeed)
                      ? accelerate(m_speed)
                      : brake(m_speed);
        switch (m_state)
        {
        case State::KeepLane:
            m_lane = currentLane;
            break;

        case State::ChangeLeft:
            m_lane = currentLane - 1;
            break;

        case State::ChangeRight:
            m_lane = currentLane + 1;
            break;
        }
    }

    double FSM::speed() const noexcept
    {
        return m_speed;
    }

    size_t FSM::lane() const noexcept
    {
        return m_lane;
    }
}