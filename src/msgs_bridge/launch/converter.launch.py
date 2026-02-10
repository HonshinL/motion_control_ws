from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动你的 C++ 转换节点
        Node(
            package='msg_converter',
            executable='scan_converter_node',
            name='scan_to_dist_bridge',
            output='screen',
            # 通过参数重映射话题（可选）
            remappings=[
                ('scan', '/rplidar_scan'),
                ('front_distance', '/robot/safety_dist')
            ]
        ),
        
        # 2. 模拟一个雷达发布者 (为了让你能直接看到效果)
        # 如果你已有真实雷达，可以删掉这部分
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='dummy_scan_publisher',
            condition=None # 这里仅作为占位示例
        )
    ])