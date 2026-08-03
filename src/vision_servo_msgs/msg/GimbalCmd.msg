# GimbalCmd.msg — Gimbal velocity-level command
std_msgs/Header header

# 端到端时延追踪。未提供的阶段保持零时间戳。
builtin_interfaces/Time source_stamp   # Sony源图像采集时刻
builtin_interfaces/Time control_stamp  # 云台快环计算完成时刻
builtin_interfaces/Time mux_stamp      # command_mux选中并发布时刻

float32 yaw_rate                   # Desired yaw angular velocity (rad/s)
float32 pitch_rate                 # Desired pitch angular velocity (rad/s)
bool hold_yaw                      # If true, hold current yaw; yaw_rate ignored
bool hold_pitch                    # If true, hold current pitch; pitch_rate ignored
