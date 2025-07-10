# deepseek_extract_location.py

from openai import OpenAI

#  替换为你的 DeepSeek API Key
client = OpenAI(
    api_key="sk-72f63fe00b7344d08088f75806831735",  # 推荐从环境变量读取
    base_url="https://api.deepseek.com"  # DeepSeek 官方地址
)

#  初始化对话上下文，\n换行，\n\n换段
conversation_history = [
    {
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
            "4. 如果只提到房间号，如“408”或“四零八”，你应从房间号中推断楼层，并返回 (4, 408)。\n"
            "5. 如果楼层与房间号中推断出的楼层不一致，以房间号推断的楼层为准。\n"
            "6. 如果输入中没有提及任何有效楼层或房间号，则返回字符串 'error'。\n\n"
            "返回格式严格如下：\n"
            "- (楼层: int, 房间号: int) 或 (楼层: int, None)\n"
            "- 或字符串 'error'（无法提取时）\n\n"
            "示例：\n"
            "“我要去三楼” → (3, None)\n"
            "“去五楼502” → (5, 502)\n"
            "“我去408” → (4, 408)\n"
            "“带我去301房” → (3, 301)\n"
            "“去三楼508” → (5, 508)\n"
            "“去四零八房间” → (4, 408)\n"
            "“你好吗” → 'error'\n"
            "“今天天气不错” → 'error'"
        )
    }
]


print("DeepSeek 语义坐标提取助手启动（输入 exit 退出）")

while True:
    user_input = input("\n你：")
    if user_input.strip().lower() == "exit":
        break

    # 添加用户输入到历史
    conversation_history.append({"role": "user", "content": user_input})

    try:
        # 调用 DeepSeek 接口
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=conversation_history,
            temperature=0.2,
            stream=False,
        )

        # 解析结果
        reply = response.choices[0].message.content.strip()
        print("AI：", reply)

        # 添加 AI 回复到历史
        conversation_history.append({"role": "assistant", "content": reply})

    except Exception as e:
        print("出错了：", e)
