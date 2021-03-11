#include "FSM.h"
#include "Route.h"
#include "Telemetry.h"

namespace
{
    double accelerationSpeed(double behaviourSpeed, double currentSpeed) noexcept
    {
        constexpr double Price = 10e4;
        double cost = 0.0;
        if (behaviourSpeed <= 10.0)
        {
            if (currentSpeed <= 10.0)
            {
                cost = -Price;
            }
            else
            {
                cost = 5.0 * Price;
            }
        }
        return cost;
    }

    struct Behaviour
    {
        using State = udacity::FSM::State;

        size_t lane;
        double speed;
        State state;
        double cost;

        Behaviour(State s, size_t currentLane, double currentSpeed,
                  const std::shared_ptr<udacity::Route> &route)
        {
            state = s;
            switch (state)
            {
            case State::Accelerate:
                lane = currentLane;
                speed = 9;
                break;

            case State::KeepLane:
                lane = currentLane;
                speed = route->maxSpeed();
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

            cost = currentCost(currentSpeed);
        }

    private:
        double currentCost(double currentSpeed) noexcept
        {
            return accelerationSpeed(speed, currentSpeed);
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
            Behaviour potentialBehaviour(nextState, m_route->frenetToLaneNumber(tm->frenet.d),
                                         tm->speed, m_route);

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
            m_speed += 0.2;
            m_lane = currentLane;
            break;

        case State::KeepLane:
            m_speed = m_route->maxSpeed(); // TODO: Use adaptive speed
            m_lane = currentLane;
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