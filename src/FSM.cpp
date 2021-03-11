#include "FSM.h"
#include "Route.h"
#include "Telemetry.h"

#include <cmath>
#include <iostream>

namespace
{
    double accelerationSpeed(double behaviourSpeed, double currentSpeed) noexcept
    {
        constexpr double Price = 10e4;
        return std::isless(currentSpeed, behaviourSpeed) ? -Price : 5.0 * Price;
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
            case State::Accelerate:
                lane = currentLane;
                speed = 10.0;
                cost = accelerationSpeed(speed, currentSpeed);
                break;

            case State::KeepLane:
                lane = currentLane;
                speed = route->maxSpeed() - 0.5;
                break;

            case State::ChangeLeft:
                speed = route->maxSpeed();
                lane = route->laneToLeft(currentLane);
                break;

            case State::ChangeRight:
                speed = route->maxSpeed();
                lane = route->laneToRight(currentLane);
                break;

            case State::Brake:
                lane = currentLane;
                speed = std::max(currentSpeed - 5.00, 0.0);
                break;
            }
        }

    private:
        double currentCost(double currentSpeed) noexcept
        {
            return cost;
        }
    };
}

namespace udacity
{
    FSM::FSM(const std::shared_ptr<Route> &route)
        : m_state(State::Accelerate),
          m_transitions({{State::Accelerate, {State::Accelerate, State::KeepLane}},
                         {State::KeepLane, {State::KeepLane, State::ChangeLeft, State::ChangeRight}},
                         {State::ChangeLeft, {State::KeepLane, State::ChangeLeft}},
                         {State::KeepLane, {State::KeepLane, State::ChangeRight}},
                         {State::Brake, {State::Brake, State::Accelerate}}}),
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
        case State::Accelerate:
            m_speed += 0.224;
            m_lane = currentLane;

            std::cout << "Accelerate" << std::endl;
            break;

        case State::KeepLane:
            m_speed = std::min(m_speed + 0.224, m_route->maxSpeed() - 0.5); // TODO: Use adaptive speed
            m_lane = currentLane;

            std::cout << "KeepLane" << std::endl;
            break;

        case State::ChangeLeft:
        case State::ChangeRight:
        case State::Brake:
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