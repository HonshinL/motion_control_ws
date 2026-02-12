


          
我来为您介绍如何通过命令行发送action目标。首先让我查看一下当前的action定义，然后提供相应的命令示例。

## 1. 查看Action定义

让我先查看我们创建的MoveToPosition action的定义：
        
            
toolName: view_files
            
status: success
          
            
filePath: /home/hongxin/Projects/10_Github/zmotion_ws/src/motion_msgs/action/MoveToPosition.action
          
基于我们创建的 `MoveToPosition` action，以下是几种通过命令行发送action目标的方法：

## 1. 使用 `ros2 action send_goal` 命令（推荐）

这是最直接的方法，可以发送action目标并实时查看反馈：

```bash
# 发送action目标
ros2 action send_goal /zmc_act/move_to_position motion_msgs/action/MoveToPosition "
{
  target_axes: [0, 1, 2, 4, 5],
  target_positions: [100.0, 200.0, 300.0, 400.0, 500.0],
  speed: 50.0,
  acceleration: 100.0,
  deceleration: 100.0,
  wait_for_completion: true
}" --feedback
```

**参数说明：**
- `target_axes`: 要移动的轴号列表 `[0, 1, 2, 4, 5]`
- `target_positions`: 对应的目标位置列表 `[100.0, 200.0, 300.0, 400.0, 500.0]`
- `speed`: 移动速度 `50.0`
- `acceleration`: 加速度 `100.0`
- `deceleration`: 减速度 `100.0`
- `wait_for_completion`: 是否等待完成 `true`
- `--feedback`: 显示实时反馈

## 2. 使用YAML文件发送（复杂目标）

对于复杂的action目标，可以创建YAML文件：

```bash
# 创建goal.yaml文件
cat > goal.yaml << 'EOF'
target_axes: [0, 1, 2, 4, 5]
target_positions: [100.0, 200.0, 300.0, 400.0, 500.0]
speed: 50.0
acceleration: 100.0
deceleration: 100.0
wait_for_completion: true
EOF

# 使用YAML文件发送
ros2 action send_goal /zmc_act/move_to_position motion_msgs/action/MoveToPosition --goal goal.yaml --feedback
```

## 3. 交互式发送（逐个输入参数）

```bash
ros2 action send_goal /zmc_act/move_to_position motion_msgs/action/MoveToPosition --interactive
```

然后按照提示逐个输入参数。

## 4. 查看Action相关信息

在发送action之前，可以先查看相关信息：

```bash
# 查看可用的action列表
ros2 action list

# 查看特定action的信息
ros2 action info /zmc_act/move_to_position

# 查看action的类型
ros2 action list -t

# 查看action的详细定义
ros2 interface show motion_msgs/action/MoveToPosition
```

## 5. 监控Action状态

在另一个终端中监控action的反馈和结果：

```bash
# 监控action反馈
ros2 topic echo /zmc_act/move_to_position/_action/feedback

# 监控action结果
ros2 topic echo /zmc_act/move_to_position/_action/result

# 监控action状态
ros2 topic echo /zmc_act/move_to_position/_action/status
```

## 6. 取消正在执行的Action

如果需要取消正在执行的action：

```bash
# 查看正在执行的action
ros2 action list --active

# 取消action（需要知道goal_id）
ros2 action cancel_goal /zmc_act/move_to_position <goal_id>
```

## 实际使用示例

**示例1：移动所有5个轴到指定位置**
```bash
ros2 action send_goal /zmc_act/move_to_position motion_msgs/action/MoveToPosition "
{
  target_axes: [0, 1, 2, 4, 5],
  target_positions: [50.0, 100.0, 150.0, 200.0, 250.0],
  speed: 30.0,
  acceleration: 50.0,
  deceleration: 50.0,
  wait_for_completion: true
}" --feedback
```

**示例2：只移动单个轴**
```bash
ros2 action send_goal /zmc_act/move_to_position motion_msgs/action/MoveToPosition "
{
  target_axes: [0],
  target_positions: [100.0],
  speed: 20.0,
  acceleration: 40.0,
  deceleration: 40.0,
  wait_for_completion: true
}" --feedback
```

## 注意事项

1. **确保节点运行**：在发送action之前，确保 `zmc_controller` 节点正在运行
2. **参数验证**：action服务器会验证参数的有效性，如果参数无效会拒绝请求
3. **实时监控**：使用 `--feedback` 参数可以实时查看移动进度
4. **安全停止**：可以使用 `Ctrl+C` 或取消命令来停止正在执行的action

这些命令应该能帮助您通过命令行发送action目标并控制设备移动到指定位置。
        