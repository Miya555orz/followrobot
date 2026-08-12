# VoiceCommand.msg
# 语音控制节点解析后的指令消息
# 支持单指令和多指令（如"跟上我，走近点"→ start_following + distance_adjust）

std_msgs/Header header

# ── 意图列表 ─────────────────────────────────────────────────
# 可能同时包含多个意图（多指令场景）
# 正式意图集合见 docs/voice_command_catalog.md。
# 执行节点只允许读取 intents，不得根据 raw_text 猜测硬件动作。
string[] intents

# ── 各意图的置信度（与 intents 一一对应） ─────────────────────
float32[] confidences

# ── ASR 原始文本 ─────────────────────────────────────────────
string raw_text

# ── 参数（各意图的细化参数，无对应参数时填默认值） ──────────────
# 距离调整相关
float32 distance         # 距离值（米），-1 表示未指定
string unit              # 单位: "m" / "cm"，空字符串表示未指定
bool distance_relative   # true=相对当前值调整，false=绝对目标/动作位移

# 角度参数（云台微调、底盘转向、环绕角）
float32 angle            # 角度值（degree），-1 表示未指定
int8 direction           # -1=右/后退，0=自动，1=左/前进

# 速度设定
string speed             # "fast" / "mid" / "slow"，空字符串表示未指定

# 切换目标相关
string target_desc       # 目标描述，如"穿红衣服的""高个子"，空字符串表示未指定

# 是否跟随模式
bool follow              # true=开启跟随，false=关闭
