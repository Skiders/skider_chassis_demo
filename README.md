# 0 you should use galactic instead of foxy!!


# 1 工程结构
├── firmware: stm32固件，根据IRbot2022电控组云台框架进行构建 https://github.com/Qylann/gimbal-standard  \
└── src \
&emsp;&emsp;├── skider_interface: skider执行层和指令层自定义通信接口    \
&emsp;&emsp;├── skider_excutor: skider执行层内部自定义通信接口  \
&emsp;&emsp;├── skider_hw: 底层通信包，存放底层接口通信节点 \
&emsp;&emsp;├── skider_sensor: 传感器处理包，存放传感器的数据处理节点   \
&emsp;&emsp;└── skider_gimbal_demo: 云台控制器DEMO包，存放了一个简单的云台控制DEMO节点  \




# 2 构建
依赖：libusblibus (https://github.com/libusb/libusb)    \
libusb安装的参考博客: https://blog.csdn.net/jiacong_wang/article/details/106720863?spm=1001.2014.3001.5502   \
注意执行单步构建时skider_hw、skider_sensor、skider_gimbal_demo依赖于skider_interface、skider_excutor

# 3 运行
在launch skider_hw之前需要赋权usb。 \
查看usb设备
```
lsusb
```
找到stm32 usb虚拟串口，假设为
```
Bus 003 Device 009: ID 0483:5740 STMicroelectronics Virtual COM Port
```
一般Bus id不会改变，而Device id在每次重新插入后都会发生变化 \
所以一般这样赋权此usb端口
```
sudo chmod 777 /dev/bus/usb/003/*
```
然后依次launch skider_hw、skider_sensor、skider_gimbal_demo


检查参数文件是否正确读取

source install/setup.bash

ros2 launch skider_hw skider_hw.launch.py

ros2 launch skider_sensor skider_sensor.launch.py

ros2 launch skider_gimbal_demo skider_gimbal_demo.launch.py