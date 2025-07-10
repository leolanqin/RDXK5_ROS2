import rclpy
from rclpy.node import Node
from deepseek_interfaces.srv import DeepseekChat
from openai import OpenAI
from std_msgs.msg import Header
#import time

client = OpenAI(
    api_key="sk-72f63fe00b7344d08088f75806831735",  # 推荐从环境变量读取
    base_url="https://api.deepseek.com"  # DeepSeek 官方地址
)

SYSTEM_PROMPT = {
    "role": "system",
    "content": (
        "你是一个语音助手，负责从用户输入中提取楼层和房间号。\n\n"
        "用户可能会表达目的地，例如：\n"
        "- “我要去五楼508”\n"
        "- “请带我去401”\n"
        "- “去三楼”\n"
        "- “送我到308房间”\n"
        "- “我想去四零八房间”\n\n"
        "你的任务是从中识别楼层和房间号，提取成格式：(楼层, 房间号)。\n\n"
        "规则如下：\n"
        "1. 房间号可以是阿拉伯数字（如 508）或中文数字（如 五零八），都要正确识别。\n"
        "2. 返回中的楼层和房间号都为整数，不保留前导 0，不加引号。\n"
        "3. 如果只提到楼层，如“去三楼”，返回 (3, None)。\n"
        "4. 如果只提到房间号，如“408”或“四零八”，你应从房间号中推断楼层，并返回 (4, 8)。\n"
        "5. 即使用户说了楼层，但只要房间号存在，应以房间号为准，将房间号前一位数字推断楼层。\n"
        "6. 如果输入中没有提及任何有效楼层或房间号，则返回字符串 'error'。\n\n"
        "返回格式严格如下：\n"
        "- (楼层: int, 房间号: int) 或 (楼层: int, None)\n"
        "- 或字符串 'error'（无法提取时）\n\n"
        "示例：\n"
        "“我要去三楼” → (3, None)\n"
        "“去五楼502” → (5, 2)\n"
        "“我去408” → (4, 8)\n"
        "“带我去301房” → (3, 1)\n"
        "“去三楼508” → (5, 8)\n"
        "“去四零八房间” → (4, 8)\n"
        "“你好吗” → 'error'\n"
        "“今天天气不错” → 'error'"
    )
}

class ExtractLocationNode(Node):
    def __init__(self):
        super().__init__('extract_location_node')
        self.srv_ = self.create_service(DeepseekChat, 'extract_location', self.extract_location_callback)
        self.get_logger().info("Extract Location Node has been started.")

    def extract_location_callback(self, request, response):
        user_input = request.position_text.strip()
        messages = [SYSTEM_PROMPT, {"role": "user", "content": user_input}]

        try:
            reply = client.chat.completions.create(
                model="deepseek-chat",
                messages=messages,
                temperature=0.2,
                stream=False
            ).choices[0].message.content.strip()

            self.get_logger().info(f"输入: {user_input}")
            self.get_logger().info(f"AI 返回: {reply}")

            # 默认失败
            response.success = False
            response.header.stamp = self.get_clock().now().to_msg()
            response.header.frame_id = "deepseek_chat"

            # 把 reply 中的所有字母转换为小写。
            if reply.lower() == "error":
                return response

            # 解析结果：(3, 8) 或 (5, None)
            import re
            match = re.match(r"\(?\s*(\d+)\s*,\s*(\d+|None)\s*\)?", reply)
            if match:
                floor = int(match.group(1))
                room_str = match.group(2)
                room = int(room_str) if room_str.lower() != "none" else -1

                response.success = True
                response.floor = floor
                response.room = room if room != -1 else 0
            else:
                self.get_logger().warn("无法解析 AI 返回格式")

        except Exception as e:
            self.get_logger().error(f"调用 DeepSeek 出错: {str(e)}")
            response.success = False

        return response

def main():
    rclpy.init()
    node = ExtractLocationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Extract Location Node has been stopped.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()