#pragma once

#include "json.hpp"

namespace udacity
{
    class SensorFusion
    {
    public:
        class Vehicle
        {
        public:
            explicit Vehicle(nlohmann::json::const_reference json) : m_data(json)
            {
            }

            int id() const { return m_data[0]; }
            double x() const { return m_data[1]; }
            double y() const { return m_data[2]; }
            double vx() const { return m_data[3]; }
            double vy() const { return m_data[4]; }
            double s() const { return m_data[5]; }
            double d() const { return m_data[6]; }

        private:
            nlohmann::json::const_reference m_data;
        };

        explicit SensorFusion(const nlohmann::json &json)
            : m_data(json[1]["sensor_fusion"])
        {
        }

        Vehicle operator[](size_t id) const
        {
            return Vehicle(m_data[id]);
        }

    private:
        nlohmann::json::const_reference &m_data;
    };
}