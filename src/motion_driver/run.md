

```
chmod +x motion_driver/*.py
colcon build --packages-select motion_driver
source install/setup.bash

ros2 run motion_driver action_server

ros2 run motion_driver action_client
```