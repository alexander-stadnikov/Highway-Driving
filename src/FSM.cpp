#include "FSM.h"
#include "Route.h"
#include "Telemetry.h"

#include <cmath>
#include <iostream>

namespace
{
    double accelerationPrice(double behaviourSpeed, double currentSpeed) noexcept
    {
        constexpr double Price = 10e4;
        return std::isless(currentSpeed, behaviourSpeed) ? -Price : 5.0 * Price;
    }

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
                  const std::shared_ptr<udacity::Route> &route)
        {
            const auto currentLane = route->frenetToLaneNumber(tm->frenet.d);
            const auto currentSpeed = tm->speed;

            cost = 0.0;

            state = s;
            switch (state)
            {
            case State::KeepLane:
                lane = currentLane;
                speed = route->recommendedSpeed();
                break;

            case State::ChangeLeft:
                speed = route->maxSpeed();
                lane = currentLane - 1;
                break;

            case State::ChangeRight:
                speed = route->maxSpeed();
                lane = currentLane + 1;
                break;
            }
        }

    private:
        double currentCost(double currentSpeed) noexcept
        {
            return accelerationPrice(speed, currentSpeed);
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

    void FSM::update(const std::shared_ptr<udacity::Telemetry> &tm) noexcept
    {
        auto minCost = std::numeric_limits<double>::max();

        for (const auto nextState : m_transitions.at(m_state))
        {
            Behaviour potentialBehaviour(nextState, tm, m_route);

            if (potentialBehaviour.cost < minCost)
            {
                minCost = potentialBehaviour.cost;
                m_state = potentialBehaviour.state;
            }
        }

        transit(tm);
    }

    void FSM::transit(const std::shared_ptr<udacity::Telemetry> &tm) noexcept
    {
        const auto currentLane = m_route->frenetToLaneNumber(tm->frenet.d);
        switch (m_state)
        {
        case State::KeepLane:
            m_speed = std::isless(m_speed, m_route->recommendedSpeed())
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