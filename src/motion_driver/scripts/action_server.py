#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from motion_driver.action import MoveToTarget

class MotionActionServer(Node):
    def __init__(self):
        super().__init__('motion_server')
        self._action_server = ActionServer(
            self, MoveToTarget, 'move_to_target', self.execute_callback)
        self.get_logger().info('Action Server 已启动，等待指令...')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'收到目标点: x={goal_handle.request.x}, y={goal_handle.request.y}')
        
        feedback_msg = MoveToTarget.Feedback()
        # 模拟“距离缩减”的过程
        for i in range(5, 0, -1):
            feedback_msg.distance_remaining = float(i)
            self.get_logger().info(f'正在移动... 剩余距离: {i}m')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0) # 模拟耗时操作

        goal_handle.succeed()
        result = MoveToTarget.Result()
        result.success = True
        self.get_logger().info('到达目标点！')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MotionActionServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()