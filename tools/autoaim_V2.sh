#! /bin/bash

# 等待10秒确保第一个节点启动
cd /home/non/Horizon_Rm_Auto_NT_M5 || exit

gnome-terminal  --  bash    /home/non/Horizon_Rm_Auto_NT_M5/tools/run_V2.sh
echo "AutoAim启动完成"

