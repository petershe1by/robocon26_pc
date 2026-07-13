# 运动控制指令
float32 linear_x       # 前进摇杆值 (-1.0~1.0, 0=停止)
float32 linear_y       # 横向摇杆值 (-1.0~1.0, 0=停止)
float32 angular_z      # 转向摇杆值 (-1.0~1.0, 0=直行)
uint8 gait_mode        # 步态模式: 0=walk, 1=trot, 2=bounds
bool enable            # 使能
bool estop             # 急停