# 运动控制指令
float32 linear_x       # 前进速度 (-1.0 ~ 1.0, 映射为目标值)
float32 linear_y       # 横向移动 (-1.0 ~ 1.0)
float32 angular_z      # 旋转角速度 (-1.0 ~ 1.0)
uint8 gait_mode        # 步态模式: 0=walk, 1=trot, 2=bounds
bool enable            # 使能
bool estop             # 急停
