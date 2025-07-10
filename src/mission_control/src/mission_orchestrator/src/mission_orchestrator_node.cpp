#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "yolo_msgs/msg/detection.hpp"
#include "elevator_interfaces/action/go_straight.hpp"
#include "mission_orchestrator/room_location_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <memory>
#include <string>

class MissionOrchestratorNode : public rclcpp::Node {
    public:
      using GoStraight = elevator_interfaces::action::GoStraight;
      using GoalHandleGoStraight = rclcpp_action::ClientGoalHandle<GoStraight>;
      using NavigateToPose = nav2_msgs::action::NavigateToPose;
      
      // 客户端对象类型，用于发送动作目标，表示一个导航目标（导航任务）的“控制句柄”，它里面封装了该目标任务的执行状态、反馈、取消、结果等内容。
      using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
     
      MissionOrchestratorNode() : Node("mission_orchestrator_node") {
        RCLCPP_INFO(this->get_logger(), "控制器节点已启动");
    
        // 初始化deepseek话题订阅器
        deepseek_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/deepseek_response", 10,
            std::bind(&MissionOrchestratorNode::deepseek_callback, this, std::placeholders::_1));
    
        // 初始化动作客户端
        go_straight_client_ = rclcpp_action::create_client<GoStraight>(this, "/go_straight");
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
        // 初始化串口通信发布器
        serial_pub_ = this->create_publisher<std_msgs::msg::String>("/serial/send", 10);
        map_switch_pub_= this->create_publisher<std_msgs::msg::String>("/mapswitch", 10);

        // 可设置默认路径,但用户也可以通过参数覆盖它；如果用户没有提供参数，就使用默认值
        yaml_path_ = this->declare_parameter<std::string>("config", 
        "/home/wheeltec/AZHSXF/src/mission_control/src/mission_orchestrator/config/room_location.yaml");
        current_floor_ = this->declare_parameter<int>("initial_floor", 4);

        // 它是 load_location_map 和 get_location 函数的内部共享状态；

        // 它的 static 修饰 限制了它只能在该源文件（.cpp）内部访问；

        // 你在 其他源文件中不能直接访问 location_map_ 本身，但你可以通过load_location_map() 和 get_location() 间接使用它。

        mission_orchestrator::load_location_map(yaml_path_);
    
      }
    
    private:
      // 成员变量（接口句柄）
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr deepseek_sub_;
      rclcpp_action::Client<GoStraight>::SharedPtr go_straight_client_;
      rclcpp::Subscription<yolo_msgs::msg::Detection>::SharedPtr yolo_sub_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_pub_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_switch_pub_;
      rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
      rclcpp::Subscription<yolo_msgs::msg::Detection>::SharedPtr former_yolo_sub_;
      rclcpp::Subscription<yolo_msgs::msg::Detection>::SharedPtr back_yolo_sub_;

      std::string yaml_path_ = "";
      int current_floor_ = 4;
      // std::string 管理的是字符序列，size() 是有效字符的数量，不包含终止符，结束符 '\0' 存在于内部，保证兼容 C 风格字符串
      std::string data_="";
    
      // 状态变量
      int target_floor_ = -1;
      int target_room_ = -1;
      bool entered_elevator_ = false;
      bool exited_elevator_ = false;

      void deepseek_callback(const std_msgs::msg::String::SharedPtr msg) {
        serial_pub_->publish(*msg);
        data_ = msg->data;
      
        if (data_.length() != 3) {
          RCLCPP_WARN(this->get_logger(), "deepseek响应格式错误，应为3位数字，收到: '%s'", data_.c_str());
          return;
        }
      
        // 解析三位数字
        int success_flag = data_[0] - '0';
        int floor = data_[1] - '0';
        int room = data_[2] - '0';

        if (success_flag != 1) {
            RCLCPP_WARN(this->get_logger(), "语音识别未成功（flag=%d），等待下次识别...", success_flag);
            return;
          }

        target_floor_ = floor;
        target_room_ = room;

        geometry_msgs::msg::PoseStamped target_pose;
        if (floor == current_floor_)
        {
          if (mission_orchestrator::get_location(floor, room, target_pose)) {
            // 使用 target_pose
            RCLCPP_INFO(this->get_logger(), "目的地位于当前楼层,目标坐标: x=%.2f, y=%.2f", 
            target_pose.pose.position.x, target_pose.pose.position.y);
            navigate_to_target(target_pose);
          }else {
            RCLCPP_WARN(this->get_logger(), "目标房间[%d-%d]未配置！", floor, room);
          }
          //deepseek_sub_.reset();
        }else {
          if (mission_orchestrator::get_location(current_floor_, 0, target_pose))
          {
            RCLCPP_INFO(this->get_logger(), "导航至%d楼电梯口", floor);
            navigate_to_target(target_pose);
          }else {
            RCLCPP_WARN(this->get_logger(), "%d号楼层电梯坐标未配置！", floor);
          }
        }
      }
      
      void navigate_to_target(const geometry_msgs::msg::PoseStamped & target_pose) {
        while (rclcpp::ok() && !nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
          RCLCPP_WARN(this->get_logger(), "等待导航服务器启动中...");
        }
        RCLCPP_INFO(this->get_logger(), "导航服务器已启动。");
      
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target_pose;
      
        RCLCPP_INFO(this->get_logger(), "已发送导航目标: x=%.2f y=%.2f",
                    target_pose.pose.position.x, target_pose.pose.position.y);

        // 是 ROS 2 中使用 Nav2 动作接口时，创建目标发送选项结构体的标准写法，SendGoalOptions 本质就是一个 “回调注册器”
        // 定义了当你调用 async_send_goal() 发送导航目标时的行为选项。
        // 目标是否接收成功后的处理（goal_response_callback）
        // 导航过程中的反馈（feedback_callback）
        // 导航是否完成的结果处理（result_callback）
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =std::bind(&MissionOrchestratorNode::nav_result_callback, this, std::placeholders::_1);
      
        nav_client_->async_send_goal(goal_msg, send_goal_options);
      }

      void nav_result_callback(const GoalHandleNav::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "导航成功，机器人已到达目标位置");
            // 此处你可以：串口发消息、开启YOLO检测电梯门等
            if (target_floor_ == current_floor_) {
              RCLCPP_INFO(this->get_logger(), "当前为同一楼层，导航至房间成功！");
              // 可发串口、执行本层任务等
              auto msg = std_msgs::msg::String();
              msg.data = data_;  // data_ 是 std::string 类型
              msg.data[0] = '4';  // 设置第一个字符为 4'，导航完成
              serial_pub_->publish(msg);
            } else {
              RCLCPP_INFO(this->get_logger(), "非当前楼层，导航至电梯口成功！");
              // 可启动YOLO检测电梯门、发串口等
              // 向STM32节点发送串口消息
              auto msg = std_msgs::msg::String();
              msg.data = data_;  // data_ 是 std::string 类型
              msg.data[0] = '2';  // 设置第一个字符为 '2'，向舵机发送按下关门按键请求
              serial_pub_->publish(msg);

              // 初始化 YOLO 检测结果订阅器
              former_yolo_sub_ = this->create_subscription<yolo_msgs::msg::Detection>(
              "/former_yolo_detector/detection", 10,
              std::bind(&MissionOrchestratorNode::former_yolo_callback, this, std::placeholders::_1));
            }
            break;
      
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "导航被中止！");
            break;
      
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "导航被取消");
            break;
      
          default:
            RCLCPP_WARN(this->get_logger(), "未知导航结果状态: %d", static_cast<int>(result.code));
            break;
        }
      }  

      void former_yolo_callback(const yolo_msgs::msg::Detection::SharedPtr msg) {
        // 检测后执行操作
        for (const auto & box : msg->boxes) {
          if (box.class_name == "open" && box.score > 0.6) {
            // 防止 YOLO 重复触发
            if (entered_elevator_) return;
            entered_elevator_ = true;

            former_yolo_sub_.reset();  // 停止订阅，释放资源
            RCLCPP_INFO(this->get_logger(), "检测到电梯门打开，分数 %.2f", box.score);

            while (rclcpp::ok() && !go_straight_client_->wait_for_action_server(std::chrono::seconds(3))) {
              RCLCPP_WARN(this->get_logger(), "等待GoStraight服务器启动中...");
            }
            RCLCPP_INFO(this->get_logger(), "GoStraight服务器已启动。");

            GoStraight::Goal goal_msg;
            goal_msg.distance = 3.0;  // 进电梯的距离，单位：米

            // 设置动作发送选项
            auto send_goal_options = rclcpp_action::Client<GoStraight>::SendGoalOptions();
            send_goal_options.result_callback =
              [this](const rclcpp_action::ClientGoalHandle<GoStraight>::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                  RCLCPP_INFO(this->get_logger(), "进电梯动作完成，移动了 3 米");

                  auto msg = std_msgs::msg::String();
                  msg.data = data_;  // data_ 是 std::string 类型
                  msg.data[0] = '3';  // 设置第一个字符为 '3'，向舵机发送按下楼层按键请求
                  serial_pub_->publish(msg);

                  back_yolo_sub_ = this->create_subscription<yolo_msgs::msg::Detection>(
                    "/back_yolo_detector/detection", 10,
                    std::bind(&MissionOrchestratorNode::back_yolo_callback, this, std::placeholders::_1));
                } else {
                  RCLCPP_WARN(this->get_logger(), "进电梯动作失败或被取消");
                }
              };
            go_straight_client_->async_send_goal(goal_msg, send_goal_options);
            break;
          }
        }
      }

      void back_yolo_callback(const yolo_msgs::msg::Detection::SharedPtr msg) {
        //int target_floor = target_floor_;
        for (const auto & box : msg->boxes) {
          try {
                int detected_floor = std::stoi(box.class_name);
                if (detected_floor == target_floor_ && box.score > 0.6) {
                    if (exited_elevator_) return;
                    exited_elevator_ = true;
                    back_yolo_sub_.reset();  // 停止订阅
                    RCLCPP_INFO(this->get_logger(), "检测到电梯已到达目标楼层 %d，分数 %.2f", detected_floor, box.score);            
                    GoStraight::Goal goal_msg;
                    goal_msg.distance = -3.0;  // 进电梯的距离，单位：米

                    // 设置动作发送选项
                    auto send_goal_options = rclcpp_action::Client<GoStraight>::SendGoalOptions();
                    send_goal_options.result_callback =
                      [this](const rclcpp_action::ClientGoalHandle<GoStraight>::WrappedResult & result) {
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {

                          entered_elevator_ = false;
                          exited_elevator_ = false;
                          RCLCPP_INFO(this->get_logger(), "出电梯动作完成，移动了 -3 米");

                          // 切换地图
                          auto msg = std_msgs::msg::String();
                          msg.data = data_;  // data_ 是 std::string 类型
                          map_switch_pub_->publish(msg);
                          rclcpp::sleep_for(std::chrono::seconds(3));

                          geometry_msgs::msg::PoseStamped target_pose;
                          if (mission_orchestrator::get_location(target_floor_, target_room_, target_pose)) {
                            RCLCPP_INFO(this->get_logger(), "目标坐标: x=%.2f, y=%.2f", 
                            target_pose.pose.position.x, target_pose.pose.position.y);
                            navigate_to_target(target_pose);
                            current_floor_ = target_floor_;
                          }else {
                            RCLCPP_WARN(this->get_logger(), "目标房间[%d-%d]未配置！", target_floor_, target_room_);
                          }
                        } else {
                          RCLCPP_WARN(this->get_logger(), "出电梯动作失败或被取消");
                        }
                      };
                      go_straight_client_->async_send_goal(goal_msg, send_goal_options);
                      break;
                  }
              } catch (const std::exception & e) {
                  RCLCPP_WARN(this->get_logger(), "无法解析检测到的类别名称: %s", box.class_name.c_str());
              }
        }
      }    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionOrchestratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
