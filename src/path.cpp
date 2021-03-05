#include "path.h"

Path::Path(const nlohmann::json &json)
    : x(json[1]["previous_path_x"]),
      y(json[1]["previous_path_y"]),
      end_s(json[1]["end_path_s"]),
      end_d(json[1]["end_path_d"])
{
}