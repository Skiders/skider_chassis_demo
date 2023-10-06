std_msgs/Header header

# 发射机构指令

# 发射机构使能，true为使能，false为无力
bool enable

# 射击指令，true为射击
# 当发射机构冷却时间结束后如果该指令为true则执行射击
bool fire

# 强制射击，如果该标志为true则无视额外冷却时间直接执行下一次射击
bool force_fire

# 最少冷却时间，实际冷却时间为最少冷却时间加上根据热量计算出的额外冷却时间
uint16 cool_down_min
