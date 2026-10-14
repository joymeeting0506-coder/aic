# 车牌识别进度：2026-10-14

- 已在 Ubuntu 24.04、ROS 2 Jazzy、Gazebo 虚拟机上完成 CPU 实测。
- 订阅 /camera/image_raw，发布 /plate_recognition/result。
- 实测识别出皖BK3P58，单帧推理约 244～287 毫秒。
- 较远视角曾把末位 8 误识别为 9，靠近并正对后识别正确。
- 自动巡检、到点停车识别、整场任务联动和结果提交仍待验收。
