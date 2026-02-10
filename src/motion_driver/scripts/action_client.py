#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from motion_driver.action import MoveToTarget
from motion_driver.zmotion import calculate_distance, get_status_label

class MotionActionClient(Node):
    def __init__(self):
        super().__init__('motion_client')
        self._action_client = ActionClient(self, MoveToTarget, 'move_to_target')

    def send_goal(self, x, y):
        goal_msg = MoveToTarget.Goal(x=x, y=y)
        self.get_logger().info('正在连接服务器...')
        self._action_client.wait_for_server()
        
        # 发送目标，并指定反馈回调函数
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被服务器拒绝')
            return
        self.get_logger().info('目标已被接收，开始执行...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # 这里的 feedback 是 Action 过程中的实时输出
        distance = feedback_msg.feedback.distance_remaining
        status = get_status_label(distance)
        self.get_logger().info(f'<<< 收到反馈:  {status} | 距离目标还有 {distance}m')

        # 用途示例 1：更新内部状态变量，供 UI 线程读取
        self.latest_distance = distance
        
        # 用途示例 2：条件触发
        if distance < 2.5:
            self.get_logger().warn('即将到达，减速慢行！')
            # 这里可以给其他节点发消息，或者改变指示灯状态

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'最终结果: {"成功" if result.success else "失败"}')
        # 任务完成后关闭节点
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MotionActionClient()
    node.send_goal(10.0, 20.0)
    total_dist = calculate_distance(10.0, 20.0)
    print(f"--- 初始总距离预估: {total_dist:.2f} ---")
    rclpy.spin(node)

if __name__ == '__main__':
    main()