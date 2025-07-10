#include "mission_orchestrator/room_location_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <map>
#include <utility>
#include <fstream>

namespace mission_orchestrator {

static std::map<std::string, geometry_msgs::msg::PoseStamped> location_map_;

bool load_location_map(const std::string & yaml_file) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_file);
    // 这个是遍历变量的定义,这表示我们要遍历 config 中的每一项
    for (const auto & entry : config) {
      // as<std::string>()：调用 YAML::Node 的模板方法 as<T>()，将该值转换为你想要的类型
      // key 是 YAML 中的键，val 是对应的值
      std::string key = entry.first.as<std::string>();
      auto val = entry.second;

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = val["frame_id"].as<std::string>();
      pose.pose.position.x = val["x"].as<double>();
      pose.pose.position.y = val["y"].as<double>();
      pose.pose.position.z = 0.0;
      double yaw = val["yaw"].as<double>();
      pose.pose.orientation.w = cos(yaw / 2);
      pose.pose.orientation.z = sin(yaw / 2);

      location_map_[key] = pose;
    }

    RCLCPP_INFO(rclcpp::get_logger("RoomLocationLoader"), "位置映射加载成功，共加载 %zu 个目标", location_map_.size());
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("RoomLocationLoader"), "加载 YAML 文件失败: %s", e.what());
    return false;
  }
}

bool get_location(int floor, int room, geometry_msgs::msg::PoseStamped & out_pose) {
  std::string key = std::to_string(floor) + "-" + std::to_string(room);
  // end() 是一个指向“末尾”的迭代器，表示“没找到”，所以：当找到了 key 时，find 不会等于 end()，就表示查找成功。在 map 中查找键（key）为 key 的元素，如果找到了，就返回对应的迭代器；如果没找到，就返回 end()。
  if (location_map_.find(key) != location_map_.end()) {
    out_pose = location_map_[key];
    return true;
  } else {
    return false;
  }
}

} // namespace mission_orchestrator
