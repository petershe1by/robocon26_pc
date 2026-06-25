# 机械臂控制指令
uint8 state            # 状态: 0=初始 1=视觉识别 2=视觉辅助定位 3=抓取 4=放置
float64 target_x       # 目标x (mm)
float64 target_y       # 目标y (mm)
float64 target_z       # 目标z (mm)
bool suction_on        # 吸盘开关
bool command_valid     # 指令是否有效
