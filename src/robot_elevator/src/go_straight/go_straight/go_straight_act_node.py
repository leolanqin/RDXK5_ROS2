import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from elevator_interfaces.action import GoStraight
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
import math

class GoStraightServer(Node):
    def __init__(self):
        #一旦有客户端调用 go_straight 动作，ROS 2 会自动调用你传入的 execute_callback(),ROS 2 会自动创建一个 ServerGoalHandle 对象，并作为参数传给这个回调。
        super().__init__('go_straight_server')
        self._action_server = ActionServer(
            self,
            GoStraight,
            'go_straight',
            self.execute_callback
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_combined', self.odom_callback, 10)
        self._current_pose = None  # 初始化当前位置

    def odom_callback(self, msg):
         self._current_pose = msg.pose.pose
    
    # 声明一个异步回调函数来处理动作请求
    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'开始执行：目标 {goal_handle.request.distance:.2f} 米')

        while self._current_pose is None:
            # 执行等待周期内所有回调请求一次
            rclpy.spin_once(self, timeout_sec=0.1)
        
        start_x = self._current_pose.position.x
        start_y = self._current_pose.position.y

        #动作接口编译后会自动生成三个类,Feedback、Result 和 Goal
        current_distance = 0.0
        speed = 0.6 
        direction = 1.0 if goal_handle.request.distance >= 0 else -1.0
        stop_twist = Twist()
        twist = Twist()
        twist.linear.x = speed * direction
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0  
        feedback_msg = GoStraight.Feedback()
        rate = self.create_rate(10)  # 10 Hz 控制频率
        result = GoStraight.Result()

        # abs() 函数用于获取绝对值，确保我们在正确的方向上前进
        try:
            while abs(current_distance) < abs(goal_handle.request.distance):

                self.cmd_vel_pub.publish(twist)
            
                # 计算距离
                dx = self._current_pose.position.x - start_x
                dy = self._current_pose.position.y - start_y
                # current_distance = (dx**2 + dy**2) ** 0.5
                current_distance = math.sqrt(dx**2 + dy**2)
                feedback_msg.current_distance = current_distance
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'当前距离: {current_distance:.2f} 米')
                # rate.sleep() 是 ROS2 的同步休眠方式
                rate.sleep()

            # 告诉 Action Server 框架：这个目标任务已成功完成
            goal_handle.succeed()
            result.success = True
            self.get_logger().info('动作完成')
            return result

        except Exception as e:
            self.get_logger().error(f'执行过程中发生异常: {e}')
            result.success = False
            return result
 
        # return 不会影响后续代码的运行
        finally:
            self.cmd_vel_pub.publish(stop_twist)
            self.get_logger().info('已发送停止指令。')

def main():
    rclpy.init()
    node = GoStraightServer()

    # 使用 MultiThreadedExecutor 以支持多线程，多个回调函数可能并发触发，用多线程执行器可以避免卡住
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()