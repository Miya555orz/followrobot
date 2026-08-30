我正在做 linux 跟拍机器人环境迁移，目标是把原项目 https://github.com/cuiangA/fcr_ros2_3 的底盘替换成逐际动力 TRON1 EDU 轮足版。我的 GitHub 仓库是 https://github.com/Miya555orz/followrobot，后续希望把迁移记录、环境配置和适配代码放进去。

当前 Linux 环境：
- Ubuntu 22.04
- ROS2 Humble
- TRON1 官方文档推荐 ROS2 Iron，但目前 Humble 已经能编译并运行仿真
- conda 已关闭自动激活，避免 colcon 调用 miniconda python

已完成：
1. 创建并编译逐际 TRON1 ROS2 workspace：
   ~/limx_ws/src 下 clone 了：
   - https://github.com/limxdynamics/limxsdk-lowlevel.git
   - https://github.com/limxdynamics/tron1-gazebo-ros2.git
   - https://github.com/limxdynamics/tron1-robot-description.git
   - https://github.com/limxdynamics/robot-visualization.git
   - https://github.com/limxdynamics/tron1-rl-deploy-ros2.git

2. 解决过的依赖问题：
   - conda python 导致缺 catkin_pkg，已通过 conda deactivate + 使用 /usr/bin/python3 解决
   - 缺 gazebo_dev，安装了 gazebo / libgazebo-dev / ros-humble-gazebo-* 相关包
   - 缺 onnxruntime_cxx_api.h，手动安装 ONNX Runtime v1.10.0 到 /usr/include 和 /usr/lib
   - 缺 xacro，安装 ros-humble-xacro
   - 缺 rqt_robot_steering，安装 ros-humble-rqt-robot-steering
   - 缺 ros2 control CLI，安装 ros-humble-ros2controlcli
   - 安装了 ripgrep

3. 编译命令：
   cd ~/limx_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

4. 编译结果：
   7 packages finished，只有 warning，没有 error。

5. 仿真启动环境变量：
   export ROBOT_TYPE=WF_TRON1A
   export RL_TYPE=isaacgym

6. 仿真启动命令：
   cd ~/limx_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch robot_hw pointfoot_hw_sim.launch.py

7. 当前状态：
   - Gazebo 中 TRON1 轮足模型已经出现
   - rqt_robot_steering 可以通过 /cmd_vel 控制机器人移动
   - /cmd_vel 类型是 geometry_msgs/msg/Twist
   - 机器人可以前进、转向、停止
   - 曾经因为旧的 ros2 topic pub / rqt / robot_hw_node 残留导致机器人跑走或重启后倒地，后来通过清理旧节点解决

8. 安全经验：
   - 不要把 /reset_world 当召回按钮，RL 控制器状态可能乱掉
   - 乱了之后最稳是停止速度发布源、杀 Gazebo/旧节点、重启 launch
   - 初期速度必须很低，建议 linear.x <= 0.10~0.15 m/s，angular.z <= 0.3~0.4 rad/s
   - 后续正式接跟拍算法前，必须写 safety_limiter 节点：
     /fcr_cmd_vel -> safety_limiter -> /cmd_vel
   - safety_limiter 需要限幅、限加速度、超时自动停车、linear.y 强制为 0

下一步任务：
1. 拉取原跟拍项目 fcr_ros2_3，分析它是否发布 /cmd_vel：
   rg "cmd_vel|Twist|geometry_msgs|publish"
2. 如果原项目直接发 /cmd_vel，需要改成发 /fcr_cmd_vel
3. 新建一个 ROS2 Python 或 C++ 包，实现 safety_limiter：
   - subscribe: /fcr_cmd_vel
   - publish: /cmd_vel
   - max_linear_x 初始 0.10 或 0.15
   - max_angular_z 初始 0.3 或 0.4
   - timeout 0.3s 没收到新速度则发 0
   - linear.y 固定 0
   - 最好支持 YAML 参数配置
4. 写 launch 文件同时启动 fcr_ros2_3 和 safety_limiter
5. 仿真验证后再考虑真机
