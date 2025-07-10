#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "stm32_map/map_loader.hpp"

class TestMapPublisher : public rclcpp::Node {
public:
  TestMapPublisher(): Node("test_map_publisher")
  {
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TestMapPublisher::publish_map, this));
  }

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid map_msg_;
  bool loaded_ = false;

  void publish_map() {
    if (!loaded_) {
      std::string yaml_path = "/home/wheeltec/AZHSXF/serial_ws/src/stm32_map/maps/floor1.yaml";
      if (!map_utils::load_map_from_yaml(yaml_path, map_msg_)) {
        RCLCPP_ERROR(this->get_logger(), "地图加载失败！");
        return;
      }
      loaded_ = true;
      RCLCPP_INFO(this->get_logger(), "地图加载完成，开始发布！");
    }
    map_pub_->publish(map_msg_);
  }
};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
