std_msgs/Header header

# 底盘移动指令

# 底盘使能，true为使能，false为无力
bool enable

# 底盘随动，true为随动
bool chassis_state_follow
# 底盘小陀螺，true为小陀螺
bool chassis_state_rotate
# 如果两者都为true或都为false，则底盘不动

# 底盘高速移动（全功率）
bool chassis_speed_fast
# 底盘慢速移动（节省功率）
bool chassis_speed_slow
# 如果两者都为true或都为false，则底盘为常规速度

# 底盘x方向上的归一化移动指令（范围：-1～1）
float64 chassis_x
# 底盘y方向上的归一化移动指令（范围：-1～1）
float64 chassis_y
