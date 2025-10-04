#!/bin/bash 
#V3版自启动脚本，用于视控一体步兵
#部署程序自启动方法：在“启动应用程序”中添加启动项，名字随便
#启动项命令为：gnome-terminal  --  bash    /home/nx/Horizon_Rm_Auto_NT_M5/tools/autostart_V3.sh
#NX风扇满速一直转
echo '1' | sudo -S jetson_clocks --fan
sec=2
cnt=0
name=Horizon_Rm_Auto_NT_M5
program_name=rm_auto
cd /home/nx/$name/build/

while [ 1 ]
do
    # 检测进程数量
    count=$(pgrep -c $program_name)
    echo "线程数量: $count"
    echo "期望数量: $cnt"
    if [ $count -gt 0 ]; then
        echo "$name 仍在运行!"
        sleep $sec
    else
        echo "启动 $name..."
        # 检查是否存在互斥锁
        if [ ! -f /tmp/$program_name.lock ]; then
            touch /tmp/$program_name.lock
            cd /home/nx/$name/build/
            ./$program_name
            rm /tmp/$program_name.lock
            echo "$name 已启动!"
            ((cnt=cnt+1))
        else
            echo "另一个 $program_name 实例正在运行."
        fi
        sleep $sec
        if [ $cnt -gt 9 ]; then
            echo "重启!"
            #reboot
        fi
    fi
done
>>>>>>> b41fc86a94dcabff32797a6432231a7ce0b99adf
