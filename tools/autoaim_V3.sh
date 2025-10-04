#! /bin/bash

# 等待10秒确保第一个节点启动
cd /home/nx/Horizon_Rm_Auto_NT_M5 || exit

gnome-terminal  --  bash    /home/nx/Horizon_Rm_Auto_NT_M5/run_V3.sh
echo "AutoAim启动完成"

