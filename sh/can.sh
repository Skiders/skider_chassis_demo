echo "m" | sudo -S ip link set can0 type can bitrate 1000000
echo "m" | sudo -S ifconfig can0 up
#echo "m" | sudo -S ifconfig eno1 192.168.0.104
echo "m" | sudo -S ip link set can1 type can bitrate 1000000
echo "m" | sudo -S ifconfig can1 up
