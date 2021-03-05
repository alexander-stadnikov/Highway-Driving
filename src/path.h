#pragma once

#include "json.hpp"

struct Path
{
    nlohmann::json::const_reference x;
    nlohmann::json::const_reference y;
    const double end_s;
    const double end_d;

    explicit Path(const nlohmann::json &json);
};