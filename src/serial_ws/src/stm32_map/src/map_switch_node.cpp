#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // ⬅️ 支持 tf2 和 geometry_msgs 互转
#include <map>
#include <string>
#include <vector>
#include <memory>

class MapSwitcherNode : public rclcpp::Node
{
public:
  MapSwitcherNode(): Node("map_switcher_node")
  {
    // 初始化地图路径与初始位姿（可改成读取 YAML）
    map_paths_ = {
      {1, "/home/wheeltec/AZHSXF/src/serial_ws/src/stm32_map/maps/floor4.yaml"},
      {2, "/home/wheeltec/AZHSXF/src/serial_ws/src/stm32_map/maps/floor6.yaml"},
      {3, "/home/user/maps/floor3.yaml"},
    };

    initial_poses_ = {
      {1, {1.0, 2.0, 0.0}},        // x, y, yaw for floor 1
      {2, { 3.5009615421295166,  1.05027437210083,  -1.54}},       // floor 2
      {3, {2.0, -1.0, 3.14}},      // floor 3
    };

    map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    local_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
    global_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/mapswitch", 10,
      std::bind(&MapSwitcherNode::map_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "地图切换节点已启动。");
  }

private:
  void map_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // if (msg->data.length() < 3) {
    //   RCLCPP_WARN(this->get_logger(), "接收到的数据长度不足3位：%s", msg->data.c_str());
    //   return;
    // }

    char floor_char = msg->data[1];
    // if (!isdigit(floor_char)) {
    //   RCLCPP_WARN(this->get_logger(), "第二位不是数字：%c", floor_char);
    //   return;
    // }

    int floor = floor_char - '0';
    if (map_paths_.find(floor) == map_paths_.end()) {
      RCLCPP_WARN(this->get_logger(), "未知楼层编号：%d", floor);
      return;
    }

    std::string map_path = map_paths_[floor];
    RCLCPP_INFO(this->get_logger(), "切换至楼层 %d，加载地图：%s", floor, map_path.c_str());

    // 调用 LoadMap 服务
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = map_path;

    while (rclcpp::ok() && !map_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "等待 map_server 服务中...");
    }

    // 异步调用 LoadMap 服务，并注册回调函数处理响应，this 捕获允许你访问类的所有成员变量和成员函数，包括 initial_poses_
    // floor 是局部变量，捕获是为了在 Lambda 里用这个具体的楼层编号
    map_client_->async_send_request(request,
      [this, floor](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future)
      {
        try {
          auto response = future.get();  // 尝试获取响应
    
          // 服务响应到了，但返回的是失败状态码
          // 从 future 中取出真正的响应结果（阻塞很短，但这在回调中是安全的）
          if (response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "地图加载失败，错误码：%d", response->result);
            return;
          }
    
          // 地图加载成功，发布初始位姿
          RCLCPP_INFO(this->get_logger(), "地图加载成功，准备发布初始位姿。");
          publish_initial_pose(initial_poses_[floor]);
          // 新增清除 costmap
          clear_costmaps();
        } catch (const std::exception &e) {
          // future.get() 过程中出错（比如服务断开）
          RCLCPP_ERROR(this->get_logger(), "调用 map_server/load_map 服务失败，异常：%s", e.what());
        }
      });    

    // auto future = map_client_->async_send_request(request);
    // 如果 3 秒之内任务 没有完成（即异步请求还没返回结果），就返回超时
    // if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    //   RCLCPP_ERROR(this->get_logger(), "等待地图加载服务响应超时！");
    //   return;
    // }
    
    // if (future.get()->result!= nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
    //   RCLCPP_ERROR(this->get_logger(), "地图加载失败：%d", future.get()->result);
    //   return;
    // }

    // RCLCPP_INFO(this->get_logger(), "地图加载成功，准备发布初始位姿。");
    // publish_initial_pose(initial_poses_[floor]);
  }

  void publish_initial_pose(const std::vector<float>& pose_data)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = this->get_clock()->now();

    pose_msg.pose.pose.position.x = pose_data[0];
    pose_msg.pose.pose.position.y = pose_data[1];

    tf2::Quaternion q;
    q.setRPY(0, 0, pose_data[2]);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    // 设置协方差矩阵
    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[7] = 0.25;
    pose_msg.pose.covariance[35] = 0.0685389;

    pose_publisher_->publish(pose_msg);
    RCLCPP_INFO(this->get_logger(), "初始位姿已发布。");
  }

  void clear_costmaps()
  {
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

    // 等待服务可用
    while (!local_clear_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "等待 /local_costmap/clear_entirely_local_costmap 服务...");
    }
    while (!global_clear_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "等待 /global_costmap/clear_entirely_global_costmap 服务...");
    }

    // 异步请求清除 local costmap
    local_clear_client_->async_send_request(request,
      [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
      {
        try {
          future.get();  // 空服务没返回字段，但出错会抛异常
          RCLCPP_INFO(this->get_logger(), "Local Costmap 清除成功！");
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Local Costmap 清除失败：%s", e.what());
        }
      });    

    // 异步请求清除 global costmap
    global_clear_client_->async_send_request(request,
      [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
      {
        try {
          future.get();  // 空服务没返回字段，但出错会抛异常
          RCLCPP_INFO(this->get_logger(), "Global Costmap 清除成功！");
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Global Costmap 清除失败：%s", e.what());
        }
      });
    
  }

  std::map<int, std::string> map_paths_;
  std::map<int, std::vector<float>> initial_poses_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapSwitcherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
