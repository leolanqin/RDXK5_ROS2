#pragma once

#include <string>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace map_utils
{

inline bool load_map_from_yaml(const std::string& yaml_path, nav_msgs::msg::OccupancyGrid& map)
{
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);
    std::string image_path = config["image"].as<std::string>();

    // 拼接图像路径，.parent_path() 是 C++17 中 std::filesystem::path 类的一个成员函数，它的作用是获取当前路径的“父目录”部分。
    std::filesystem::path yaml_dir = std::filesystem::path(yaml_path).parent_path();
    std::string full_image_path = (yaml_dir / image_path).string();

    // 读取yaml文件中的地图配置
    double resolution = config["resolution"].as<double>();
    std::vector<double> origin = config["origin"].as<std::vector<double>>();
    int negate = config["negate"].as<int>();
    double occupied_thresh = config["occupied_thresh"].as<double>();
    double free_thresh = config["free_thresh"].as<double>();

    // 加载图像
    cv::Mat image = cv::imread(full_image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      std::cerr << "无法加载地图图像: " << full_image_path << std::endl;
      return false;
    }

    // 构造地图信息
    map.info.resolution = resolution;
    map.info.width = image.cols;
    map.info.height = image.rows;

    map.info.origin.position.x = origin[0];
    map.info.origin.position.y = origin[1];
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    map.data.resize(map.info.width * map.info.height);

    // 这是标准的 双层遍历图像每个像素点，从左到右、从上到下
    for (int y = 0; y < image.rows; ++y) {
      for (int x = 0; x < image.cols; ++x) {
        // 从 cv::Mat 图像中读取第 (y, x) 像素的灰度值（范围 0~255）。
        uint8_t pixel = image.at<uint8_t>(y, x);
        // OpenCV 图像 y=0 在 上面
        // ROS OccupancyGrid y=0 在 下面
        // 所以要把 y 倒过来（翻转Y轴），使得图像的坐标和ROS地图坐标一致。
        // 下标二维变成一维
        int idx = (image.rows - 1 - y) * image.cols + x;  // y轴翻转

        std::cout << "当前 pixel 值为: " << static_cast<int>(pixel) << std::endl;
        // 这句是将像素值归一化
        double scale = pixel / 255.0;

        // 根据 negate 参数决定是否反转占用值
        if (negate==0) scale = 1.0 - scale;

        if (scale > occupied_thresh)
          map.data[idx] = 100;       // 占用
        else if (scale < free_thresh)
          map.data[idx] = 0;         // 空闲
        else
          map.data[idx] = -1;        // 未知
      }
    }

    map.header.stamp = rclcpp::Clock().now();
    map.header.frame_id = "map";
    return true;
  } catch (const std::exception& e) {
    std::cerr << "读取地图失败: " << e.what() << std::endl;
    return false;
  }
}

} // namespace map_utils
