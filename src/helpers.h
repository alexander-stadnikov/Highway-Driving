#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "json.hpp"

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

struct Waypoint
{
    double x;
    double y;
    double s;
    double dx;
    double dy;

    explicit Waypoint(const std::string &data)
    {
        std::istringstream iss(data);
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> dx;
        iss >> dy;
    }
};
struct Map
{
    std::vector<Waypoint> waypoints;

    explicit Map(const std::string &csv)
    {
        std::ifstream in(csv.c_str(), std::ifstream::in);
        std::string line;

        while (getline(in, line))
        {
            waypoints.push_back(Waypoint(line));
        }
    }
};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
std::string
hasData(const std::string &s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const Map &map);