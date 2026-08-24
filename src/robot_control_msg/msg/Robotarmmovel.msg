# 目标位姿（左臂）
float64 lx
float64 ly
float64 lz
# 左臂姿态可二选一：ZYX欧拉角 or 四元数，若四元数均为0则使用欧拉角
float64 lroll
float64 lpitch
float64 lyaw
float64 lqx
float64 lqy
float64 lqz
float64 lqw

# 目标位姿（右臂）
float64 rx
float64 ry
float64 rz
float64 rroll
float64 rpitch
float64 ryaw
float64 rqx
float64 rqy
float64 rqz
float64 rqw

# 规划限制
float64 vel
float64 acc

