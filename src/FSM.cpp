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

            if (std::isgreater(dst, safetyDst))
            {
                return speedLimit;
            }
            else if (std::isgreater(dst, 2.0 * safetyDst / 3.0))
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

        double keepLane(int currentLane) const noexcept
        {
            return 10e2 * (lane == currentLane ? -10.0 : 0.0);
        }

        double stayOnTheRoad(int numberOfLanes) const noexcept
        {
            return lane < 0 || lane >= numberOfLanes
                       ? 5 * 10e6
                       : 0.0;
        }

        double overtake(int currentLane, double dst, double lim, double laneSpeed) const noexcept
        {
            if (std::isless(dst, 75) && laneSpeed != udacity::SensorFusion::Unlimited)
            {
                return 10.0 * (lim - laneSpeed) * 10e2;
            }

            if (currentLane > lane)
            {
                return -100.0;
            }
            else if (currentLane < lane)
            {
                return -50.0;
            }

            return 0.0;
        }

        static double avoidCollision(double front, double back, double safety) noexcept
        {
            double cost = 0.0;
            if (std::isless(front, 10))
            {
                cost += 10e6;
            }
            else if (std::isless(front, safety * .25))
            {
                cost += 2 * 10e6;
            }

            if (std::isless(back, 10.0))
            {
                cost += 10e6;
            }

            return cost;
        }

        static double takeFreeLane(double front) noexcept
        {
            return -5.0 * std::min(75.0, front) * 10e2;
        }

        double keepUntilFast(double currentSpeed,
                             int currentLane) const noexcept
        {
            return std::isless(currentSpeed, 25.0) && lane == currentLane
                       ? -5.0 * 10e6
                       : 0.0;
        }

        double calculateCost(const std::shared_ptr<udacity::Telemetry> &tm,
                             const std::shared_ptr<udacity::Route> &route,
                             const udacity::SensorFusion &sf) const noexcept
        {
            const auto currentLane = route->frenetToLaneNumber(tm->frenet.d);

            const auto dstInFront = sf.freeDistanceInFront(lane);
            const auto dstBehind = sf.freeDistanceBehind(lane);

            const double speedLimit = route->maxSpeed();
            double safetyDst = safetyDistance(route->maxSpeed());

            return keepLane(currentLane) +
                   stayOnTheRoad(route->numberOfLanes()) +
                   overtake(currentLane, dstInFront, speedLimit, sf.speedOfVehicleInFront(lane)) +
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

            if (std::isless(potentialBehaviour.cost, minCost))
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
        m_previousSpeed = m_speed;
        m_speed = std::isless(m_speed, safeSpeed)
                      ? accelerate(m_speed)
                      : brake(m_speed);

        double dV = m_speed - m_previousSpeed;
        double dT = 0.02;
        double A = dV / dT;
        double J = A / dT;
        const double maxA = 9.0;
        const double maxJ = 9.0;
        if (std::isgreaterequal(std::fabs(A), maxA) || std::isgreaterequal(std::fabs(J), maxJ))
        {
            const double K = maxA / std::fabs(A);
            dV *= K;
            m_speed = m_previousSpeed + dV;
        }

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