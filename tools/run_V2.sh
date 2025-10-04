#! /bin/bash

# 等待10秒确保第一个节点启动
cd /home/non/Horizon_Rm_Auto_NT_M5/build
#老串口模块
echo '1' | sudo -S chmod 666 /dev/ttyUSB0
echo '1' | sudo -S chmod 666 /dev/ttyUSB1
#飞机的达妙NX新载板用ttyTHS0
echo '1' | sudo -S chmod 666 /dev/ttyTHS0
echo '1' | sudo -S chmod 666 /dev/ttyTHS1
#英雄，哨兵和串口版步兵用ttyACM0
echo '1' | sudo -S chmod 666 /dev/ttyACM0
echo '1' | sudo -S chmod 666 /dev/ttyACM1
./rm_auto
echo "AutoAim启动完成"

