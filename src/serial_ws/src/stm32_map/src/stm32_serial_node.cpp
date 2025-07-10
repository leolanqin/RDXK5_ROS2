#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>

class STM32SerialNode : public rclcpp::Node {
public:
  STM32SerialNode() : Node("stm32_serial_node") {
    this->declare_parameter<std::string>("port", "/dev/ttyS1");
    this->declare_parameter<int>("baudrate", 115200);

    std::string port;
    int baud;

    this->get_parameter("port", port);
    this->get_parameter("baudrate", baud);

    try {
      serial_port_.setPort(port);
      serial_port_.setBaudrate(baud);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      serial_port_.setTimeout(to);
      serial_port_.open();
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "串口打开失败: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    if (serial_port_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "串口打开成功: %s, 波特率: %d", port.c_str(), baud);
    }

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/serial/send", 10,
      std::bind(&STM32SerialNode::send_callback, this, std::placeholders::_1));
  }

private:
  void send_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (serial_port_.isOpen()) {
      serial_port_.write(msg->data);
      RCLCPP_INFO(this->get_logger(), "发送串口消息: '%s'", msg->data.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "串口未打开，无法发送！");
    }
  }

  serial::Serial serial_port_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<STM32SerialNode>());
  rclcpp::shutdown();
  return 0;
}
