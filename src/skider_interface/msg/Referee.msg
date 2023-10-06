std_msgs/Header header

# 裁判系统部分信息

# 机器人ID，00x表示红方阵营id为x，10x表示蓝方阵营id为x
uint8 robot_id

# 机器人电源状态
# 底盘电源状态，true为上电，false为断电
bool robot_chassis_power
# 云台电源状态，true为上电，false为断电
bool robot_gimbal_power
# 发射机构电源状态，true为上电，false为断电
bool robot_ammo_power

# 机器人剩余血量
uint16 robot_blood_remain
# 机器人最大血量
uint16 robot_blood_max

# 机器人上限射速
uint16 shoot_speed_max

# 部分敌方阵营机器人剩余血量
# 敌方英雄剩余血量
uint16 enemy1_blood_remain
# 敌方3号步兵剩余血量
uint16 enemy3_blood_remain
# 敌方4号步兵剩余血量
uint16 enemy4_blood_remain
# 敌方5号步兵剩余血量
uint16 enemy5_blood_remain
