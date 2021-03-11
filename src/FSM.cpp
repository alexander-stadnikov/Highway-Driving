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
            const auto currentSpeed = tm->speed;

            cost = 0.0;

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
        }

    private:
        double expectedSpeed(const std::shared_ptr<udacity::Route> &route,
                             const udacity::SensorFusion &sf) const noexcept
        {
            const double dst = sf.freeDistanceInFront(lane);
            const double speedInFront = sf.speedOfVehicleInFront(lane);
            const double safetyReactionTime = 1.5;
            const double speedLimit = route->maxSpeed() - 0.2;
            const double safetyDst = std::min(safetyReactionTime * speedLimit,
                                              safetyReactionTime * speedInFront);
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

        double currentCost(double currentSpeed) const noexcept
        {
            return cost;
        }
    };
}

namespace udacity
{
    FSM::FSM(const std::shared_ptr<Route> &route)
        : m_state(State::KeepLane),
          m_transitions({{State::KeepLane, {State::KeepLane, State::ChangeLeft, State::ChangeRight}},
                         {State::ChangeLeft, {State::KeepLane, State::ChangeLeft}},
                         {State::KeepLane, {State::KeepLane, State::ChangeRight}}}),
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
        switch (m_state)
        {
        case State::KeepLane:
            m_speed = std::isless(m_speed, safeSpeed)
                          ? accelerate(m_speed)
                          : brake(m_speed);
            m_lane = currentLane;
            break;

        case State::ChangeLeft:
        case State::ChangeRight:
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