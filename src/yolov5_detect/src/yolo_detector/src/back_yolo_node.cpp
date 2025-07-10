#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>

#include "yolo_msgs/msg/detection.hpp"
#include "yolo_msgs/msg/bounding_box.hpp"
#include "std_msgs/msg/header.hpp"

#include "yolo_detector/bpu_detect.hpp"  // 你的推理类

class BackYoloDetectorNode : public rclcpp::Node {
    public:
    BackYoloDetectorNode()
        : Node("back_yolo_detector_node") {
            // 初始化 BPU 推理器
            detector_ = std::make_shared<BPU_Detect>();
            if (!detector_->Init()) {
                RCLCPP_FATAL(this->get_logger(), "检测器初始化失败");
                rclcpp::shutdown();
                return;
            }

            // 订阅图像
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "back_usbcam/image_raw", 10,
                std::bind(&BackYoloDetectorNode::image_callback, this, std::placeholders::_1));

            // 发布识别结果
            result_pub_ = this->create_publisher<yolo_msgs::msg::Detection>("/back_yolo_detector/detection", 10);

            RCLCPP_INFO(this->get_logger(), "BACK_YOLO识别节点启动成功");
        }

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                // ROS图像 → OpenCV图像
                cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

                // 运行模型检测
                cv::Mat output_img;
                bool success = detector_->Detect(frame, output_img);

                if (success) {
                    std::vector<std::string> detected_classes;

                    // 收集当前帧中所有检测到的类别
                    for (size_t cls_id = 0; cls_id < detector_->GetClassNames().size(); ++cls_id) {
                        for (int i : detector_->GetIndices()[cls_id]) {
                            detected_classes.push_back(detector_->GetClassNames()[cls_id]);
                        }
                    }

                    if (!detected_classes.empty()) {
                        // 只取第一个（或可统计最多的类别）用于稳定判断（检测到目标物体并且画面稳定，而不是连续检测到同一个目标）
                        std::string current_class = detected_classes[0];

                        if (current_class == last_detected_class_) {
                            stable_count_++;
                        } else {
                            last_detected_class_ = current_class;
                            stable_count_ = 1;
                        }

                        // 达到稳定阈值才发布
                        if (stable_count_ >= STABLE_THRESHOLD) {
                            yolo_msgs::msg::Detection detection_msg;
                            detection_msg.header.stamp = msg->header.stamp;
                            detection_msg.header.frame_id = msg->header.frame_id;

                            for (size_t cls_id = 0; cls_id < detector_->GetClassNames().size(); ++cls_id) {
                                for (int i : detector_->GetIndices()[cls_id]) {
                                    const auto& box = detector_->GetBBoxes()[cls_id][i];
                                    float score = detector_->GetScores()[cls_id][i];

                                    yolo_msgs::msg::BoundingBox bbox;
                                    bbox.class_name = detector_->GetClassNames()[cls_id];
                                    bbox.score = score;
                                    bbox.xmin = box.x;
                                    bbox.ymin = box.y;
                                    bbox.xmax = box.x + box.width;
                                    bbox.ymax = box.y + box.height;

                                    detection_msg.boxes.push_back(bbox);
                                }
                            }

                            result_pub_->publish(detection_msg);
                            RCLCPP_INFO(this->get_logger(), "已发布 %lu 个目标（类别: %s，稳定 %d 次）", 
                                        detection_msg.boxes.size(), current_class.c_str(), stable_count_);

                            // 发布后重置，避免重复连续发
                            stable_count_ = 0;
                            last_detected_class_.clear();
                        } else {
                            RCLCPP_INFO(this->get_logger(), "检测到 %s，但未达到稳定阈值(%d/%d)", 
                                        current_class.c_str(), stable_count_, STABLE_THRESHOLD);
                        }
                    } else {
                        // 本帧检测成功但无有效目标（空结果）
                        RCLCPP_WARN(this->get_logger(), "检测成功但无有效目标");
                        stable_count_ = 0;
                        last_detected_class_.clear();
                    }

                } else {
                    RCLCPP_WARN(this->get_logger(), "检测失败");
                    stable_count_ = 0;
                    last_detected_class_.clear();
                }

            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "图像转换错误: %s", e.what());
            }
        }
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<yolo_msgs::msg::Detection>::SharedPtr result_pub_;
        std::shared_ptr<BPU_Detect> detector_;
        std::string last_detected_class_;
        int stable_count_ = 0;
        const int STABLE_THRESHOLD = 1;  // 连续稳定次数阈值（可调）
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BackYoloDetectorNode>());
    rclcpp::shutdown();
    return 0;
}