#pragma once

#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mission_orchestrator {

// 加载 YAML 中的位姿坐标
bool load_location_map(const std::string & yaml_file);

// 查找坐标
bool get_location(int floor, int room, geometry_msgs::msg::PoseStamped & out_pose);

} // namespace mission_orchestrator

