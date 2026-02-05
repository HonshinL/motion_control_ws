


```
g++ src/main.cpp src/zmcaux.cpp \
    -I include \
    -L lib \
    -lzmotion -lpthread \
    -o zmotion_test
```

```
LD_LIBRARY_PATH=./lib ./zmotion_test 192.168.0.11
LD_LIBRARY_PATH=./lib ./get_controller_info 192.168.0.11
```

ros2 run zmotion_driver zmotion_test

ros2 service call /zmc_srv/convert_dxf_to_xml motion_msgs/srv/ConvertDxfToXml "{dxf_file_path: '/home/hongxin/Projects/10_Github/zmotion_ws/src/zmotion_driver/config/example.dxf'}"

ros2 topic echo /zmc_status

# 发送action目标
ros2 action send_goal /zmc/move_to_position motion_msgs/action/MoveToPosition "
{
  target_axes: [0, 1, 2, 4, 5],
  target_positions: [50.0, 100.0, 150.0, 200.0, 250.0],
  speed: 30.0,
  acceleration: 50.0,
  deceleration: 50.0,
  wait_for_completion: true
}" --feedback

ros2 action send_goal /zmc/move_to_position motion_msgs/action/MoveToPosition "
{
  target_axes: [0],
  target_positions: [100.0],
  speed: 20.0,
  acceleration: 40.0,
  deceleration: 40.0,
  wait_for_completion: true
}" --feedback

./install/zmotion_driver/lib/zmotion_driver/zmotion_test 0 100.0

// 1. 先建立永久的跟随关系（通常在初始化执行一次即可）
ZAux_Direct_Single_Addax(g_handle, 1, 0, 1.0); // 轴1 永远跟着 轴0 跑

// 2. 运动时，只需要控制“主轴”
ZAux_Direct_Single_MoveAbs(g_handle, 0, 100.0); // 轴0动，轴1自动跟