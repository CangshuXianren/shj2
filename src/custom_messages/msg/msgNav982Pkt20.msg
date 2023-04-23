Header header

##   0. 接收到 NAV982 的 ID=20 数据包个数
uint32 NAV_Count

##   1. 系统状态
uint16 NAV_System_status

##   2. 滤波器状态
uint16 NAV_Filter_status

##   3. Unix 时间（秒）
uint32 NAV_Unix_time

##   4. Unix 时间（微秒）
uint32 NAV_Microseconds

##   5. 纬度信息（rad）
float64 NAV_Latitude

##   6. 经度信息（rad）
float64 NAV_Longitude

##   7. 高度信息（m）
float64 NAV_Altitude

##   8. 北向速度（m/s）
float32 NAV_Velocity_North

##   9. 东向速度（m/s）
float32 NAV_Velocity_East

##  10. 垂向速度（m/s）
float32 NAV_Velocity_Vertical

##  11. X 轴加速度（m/s/s）
float32 NAV_Acceleration_x

##  12. Y 轴加速度（m/s/s）
float32 NAV_Acceleration_y

##  13. Z 轴加速度（m/s/s）
float32 NAV_Acceleration_z

##  14. 重力加速度（g）
float32 NAV_G

##  15. 横滚角度（rad）
float32 NAV_Roll

##  16. 俯仰角度（rad）
float32 NAV_Pitch

##  17. 航向角度（rad）
float32 NAV_Heading

##  18. X 轴角速度（rad/s）
float32 NAV_Angular_Velocity_x

##  19. Y 轴角速度（rad/s）
float32 NAV_Angular_Velocity_y

##  20. Z 轴角速度（rad/s）
float32 NAV_Angular_Velocity_z

##  21. 纬度标准差（m）
float32 NAV_Latitude_Standard_Deviation

##  22. 经度标准差（m）
float32 NAV_Longitude_Standard_Deviation

##  23. 高度标准差（m）
float32 NAV_Altitude_Standard_Deviation



