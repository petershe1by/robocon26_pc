# 任务状态
string status          # IDLE / MATH_SOLVING / NAVIGATING / GRASPING / PLACING / COMPLETED / ERROR
uint32 current_target  # 当前目标物资箱编号
uint32 blocks_delivered  # 已送达物资箱数量
uint32 blocks_remaining  # 剩余物资箱数量
int32 score            # 当前得分
bool high_zone_known   # 是否已确定高分区
