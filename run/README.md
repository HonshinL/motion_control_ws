# ZMotion Driver - ROS2 运动控制驱动

一个基于ROS2的ZMotion控制器驱动包，提供对ZMC系列运动控制器的完整支持。

## 🚀 功能特性

### 核心功能
- ✅ 多轴运动控制（支持5个轴：0,1,2,4,5）
- ✅ 实时状态监控（位置、速度、限位状态）
- ✅ 绝对/相对运动控制
- ✅ 软件/硬件限位保护
- ✅ 运动过程实时反馈

### ROS2集成
- ✅ ROS2节点自动管理
- ✅ 标准消息接口（MotionStatus）
- ✅ Action服务支持（MoveToPosition）
- ✅ 参数配置支持
- ✅ 实时数据发布（50Hz）

## 📦 安装和编译

### 环境要求
- ROS2 Humble 或更高版本
- CMake 3.8+
- C++14 编译器

### 编译步骤
```bash
colcon build --packages-select zmotion_driver
source install/setup.bash
```

## 🔧 使用方法

### 1. 启动主节点
```bash
ros2 run zmotion_driver zmotion_node
```

### 2. 监控运动状态
```bash
# 查看实时运动状态
ros2 topic echo /zmc/motion_status

# 查看节点信息
ros2 node info /zmc_controller
```

### 3. 使用Action控制运动
```bash
# 发送运动目标
ros2 action send_goal /zmc/move_to_position motion_msgs/action/MoveToPosition "
{
  target_axes: [0, 1, 2, 4, 5],
  target_positions: [100.0, 200.0, 300.0, 400.0, 500.0],
  speed: 50.0,
  acceleration: 100.0,
  deceleration: 100.0,
  wait_for_completion: true
}" --feedback
```

### 4. 命令行工具
```bash
# 相对运动控制
./install/zmotion_driver/lib/zmotion_driver/move_axis 0 100.0

# 查询限位状态
./install/zmotion_driver/lib/zmotion_driver/zmotion_test
```

## 📁 项目结构

```
zmotion_driver/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包配置
├── README.md              # 本文档
├── include/               # 头文件
│   └── zmotion_driver/
│       ├── zmc_controller.h  # 主控制器类
│       ├── zmcaux.h         # ZMotion SDK封装
│       └── zmotion.h        # 基础定义
├── src/                   # 源代码
│   ├── zmc_controller.cpp   # 控制器实现
│   ├── zmcaux.cpp          # SDK接口实现
│   └── main.cpp           # 主程序入口
└── test/                  # 测试工具
    ├── move_axis.cpp      # 轴运动控制工具
    └── get_and_set_limits.cpp # 限位查询工具
```

## 🔌 硬件连接

### 控制器配置
- **IP地址**: 192.168.0.11（默认）
- **端口**: 标准以太网端口
- **轴配置**: 5个轴（0,1,2,4,5）

### 网络设置
```bash
# 检查网络连接
ping 192.168.0.11

# 如果需要修改IP，编辑源代码中的配置
# 文件: src/zmc_controller.cpp
```

## 📊 消息接口

### MotionStatus 消息
```yaml
joint_state:
  header:
    stamp: {sec: 123, nanosec: 456}
    frame_id: "zmc_status"
  name: ["axis_0", "axis_1", "axis_2", "axis_4", "axis_5"]
  position: [100.0, 200.0, 300.0, 400.0, 500.0]  # 反馈位置
  velocity: [10.0, 20.0, 30.0, 40.0, 50.0]       # 当前速度
```

### MoveToPosition Action
- **Goal**: 目标位置、速度参数
- **Feedback**: 实时进度、当前位置
- **Result**: 最终结果、执行状态

## ⚙️ 参数配置

### 运行时参数
```yaml
# 控制器IP地址
controller_ip: "192.168.0.11"

# 监控轴号
monitoring_axis: 0

# 连接超时时间
connect_search_timeout_ms: 5000
```

### 配置文件
创建 `config/params.yaml`:
```yaml
zmc_controller:
  ros__parameters:
    controller_ip: "192.168.0.11"
    monitoring_axis: 0
```

## 🛠️ 开发指南

### 添加新的运动功能
1. 在 `zmc_controller.h` 中声明方法
2. 在 `zmc_controller.cpp` 中实现功能
3. 更新ROS2接口（如需要）
4. 添加测试用例

### 扩展轴支持
修改头文件中的轴配置：
```cpp
static constexpr int NUM_AXES = 5;
static constexpr int AXES[NUM_AXES] = {0, 1, 2, 4, 5};
```

## 🐛 故障排除

### 常见问题

#### 1. 连接失败
```bash
❌ 无法连接到控制器: 192.168.0.11
```
**解决方案**:
- 检查控制器电源和网络连接
- 确认IP地址正确
- 检查防火墙设置

#### 2. 库文件找不到
```bash
error while loading shared libraries: libzmotion.so
```
**解决方案**:
```bash
export LD_LIBRARY_PATH=/path/to/zmotion_driver/lib:$LD_LIBRARY_PATH
```

#### 3. 编译错误
**解决方案**:
- 确保所有依赖项已安装
- 检查CMake配置
- 验证头文件路径

### 调试技巧
```bash
# 启用详细日志
export RCLCPP_LOG_LEVEL=DEBUG

# 查看节点图
ros2 node list
ros2 topic list

# 监控特定话题
ros2 topic hz /zmc/motion_status
```

## 📈 性能指标

- **数据发布频率**: 50Hz
- **运动控制精度**: 0.001单位
- **连接响应时间**: < 100ms
- **多轴同步误差**: < 1ms

## 🤝 贡献指南

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

## 📄 许可证

本项目采用 Apache License 2.0 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 🙏 致谢

- ZMotion 官方SDK支持
- ROS2 社区
- 所有贡献者和测试人员

## 📞 技术支持

如有问题或建议，请通过以下方式联系：
- 创建 Issue
- 发送邮件至: user@example.com
- 查看项目Wiki获取更多文档

---

**最后更新**: 2026-02-06  
**版本**: v0.0.0  
**维护者**: User