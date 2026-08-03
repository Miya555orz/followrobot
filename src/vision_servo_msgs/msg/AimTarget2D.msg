# AimTarget2D.msg — 单一人脸/上身/关键点瞄准点，像素级精确指示
# 所有坐标均为 Sony 原始图像像素坐标，保留源图像的 stamp 和 frame_id

std_msgs/Header header

# 源类型枚举
uint8 FACE=0               # 人脸检测框中心
uint8 KEYPOINT=1           # 身体关键点（如鼻尖）
uint8 PREDICTED_FACE=2     # 根据预测/卡尔曼滤波推得的人脸位置
uint8 UPPER_BODY=3         # 上身框中心
uint8 LOST_PREDICTION=4    # 目标丢失后的预测位置

# 新二维快环使用的语义化来源。保留上面的旧枚举，避免破坏现有节点。
uint8 HEAD=5               # 稳定的人头关注点
uint8 SHOULDERS=6          # 头肩关注点（当前默认）
uint8 BODY_BBOX=7          # 人体检测框派生关注点
uint8 PREDICTED=8          # 状态估计器显式传播的关注点

# 关联的跟踪 ID
int32 tracking_id

# 像素坐标（Sony 原图坐标系）
float32 pixel_x
float32 pixel_y

# 图像平面关注点速度（pixel/s）。它不是人物相对世界的真实速度。
float32 pixel_velocity_x
float32 pixel_velocity_y

# 当前像素估计的不确定度（pixel^2）。
float32 covariance_x
float32 covariance_y

# 状态估计完成时刻；header.stamp 始终保留 Sony 源图像采集时刻。
builtin_interfaces/Time estimate_stamp

# 置信度 [0.0, 1.0]
float32 confidence

# 数据来源类型
uint8 source

# true 表示当前坐标来自无新测量的短时传播。
bool predicted

# 有效性标志（false 表示当前帧无有效观测）
bool valid
