## behaviour tree implementation
---

For testing, run these commands:
```bash

ros2 topic pub -1 /bt_demo/safety/estop std_msgs/msg/Bool "{data: false}"
ros2 topic pub -1 /bt_demo/battery/level std_msgs/msg/Float32 "{data: 0.6}"
ros2 topic pub -1 /bt_demo/task/has_task std_msgs/msg/Bool "{data: true}"

ros2 topic pub -1 /bt_demo/battery/level std_msgs/msg/Float32 "{data: 0.1}"
ros2 topic pub -1 /bt_demo/safety/estop std_msgs/msg/Bool "{data: true}"
```
