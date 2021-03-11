#pragma once

#include <unordered_map>
#include <vector>
#include <memory>

namespace udacity
{
    class Telemetry;
    class Route;

    class FSM
    {
    public:
        enum class State
        {
            Accelerate,
            Brake,
            KeepLane,
            ChangeLeft,
            ChangeRight
        };

    public:
        explicit FSM(const std::shared_ptr<Route> &route);

        void update(const std::shared_ptr<udacity::Telemetry> &) noexcept;
        double speed() const noexcept;
        size_t lane() const noexcept;

    private:
        void transit(const std::shared_ptr<udacity::Telemetry> &) noexcept;

    private:
        State m_state;
        const std::unordered_map<State, std::vector<State>> m_transitions;
        const std::shared_ptr<Route> &m_route;
        double m_speed;
        size_t m_lane;
    };
}