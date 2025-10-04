#ifndef DEFINE_H
#define DEFINE_H

//加速推理框架：二选一
#define TRT //TensorRT，适用于有Nvidia显卡的设备
//#define OV  //OpenVINO，适用于所有设备

// 相机选择:
//#define MIDVISION //英雄，飞机
//#define MDAUTOEXPO //迈德自动曝光（往往会过曝，不建议启用）
//#define DAHENG1 //全向步兵
#define DAHENG2 //串口版步兵
//#define HK //哨兵
//#define GENERIC //通过OpenCV默认api调用通用相机,无畸变校正，仅用于无工业相机时临时测试
//#define VIDEO

// 功能选择:
#define Img_Show //程序图形界面，正式上车时禁用以提升运行速度
#define UI_Show //是否绘制参数UI
//#define S_Frame //手动单帧调试模式
//#define Save_Record //保存视频内录，关闭UI_Show时即可进入采集模式，用于比赛时进行素材采集（此时不会采集UI）
//#define Nova_Get_Img //新版获取相机图像方法（海康相机必须启用，全向需禁用，其他相机无所谓） 
#define Get_Debug //在终端中显示所有接收到的16进制数据，仅用于串口通信模式
//#define Time_Sync_Serial //串口通信时间同步功能（实验性功能，目前已知启用会导致串口通信出现无法接收的问题）
//#define Last_Pose //显示上一帧预测（追踪红点的黄点）
//#define Veh_Estim //整车估计(实验性功能且功能未完成)
//#define Plane //飞机模式
//#define Outpost //击打前哨站（测试性功能）

//#define Edge_Track_Mode   //边缘追踪模式（用于解决在边缘上追踪失败的问题，通过在边缘上停止预测实现）
#define Pure_Track_Mode   //纯追踪模式 
//#define Pure_Predict_Mode //纯预测模式
#define Auto_Fire_Normal //普通自动开火
//#define Inter_Vis_Ctl //视控一体通信模式（用于全向步兵）
//#define CPU_Thread_ID_Lock //为每个线程锁定固定的cpu核心（视控一体通信模式必须启用）
//#define Nova_CeresVelocity //改造最小二乘法（实验性功能，全向步兵需启用）
//#define Debug_Vofa //Vofa调试波形参数（仅用于视控一体通信）
//#define Pit_Lock //将p轴最大发送值限制在20以内，用于全向步兵
//#define Serial_Self_Withering 
//带自凋零机制的串口通信模式（用于解决包头包尾被异常修改的问题，识别到错误包头时自动停止程序，仅用于串口通信）

//#define Test1//是否启用now_v[2] = 0，RTM2之前未启用，但不启用可能出现小幅度斜着预测的怪异问题
#define Test2//是否启用last_time_ = current_time_; //Omni_V3启用
//#define Test3//是否启用cam2ptz（默认不启用）

//#define Gyro1//反陀螺第一阶段开关
//#define Gyro2//反陀螺第二阶段开关
//#define Gyro_UI //反陀螺功能UI（实验性功能，需配合其他陀螺功能）
//#define Auto_Fire_Gyro //反陀螺自动开火（需同时启用Gyro1和2才有效）
//#define NEXT_Gyro //新版反陀螺

//#define NT //新预测内核，以下功能皆依赖于NT内核
//#define Better_Target //根据装甲板检查是否需要重新初始化
//#define NT_Gyro //NT的反陀螺，思路类似旧版反陀螺，目前存在问题，无法使用
//#define Best_Yaw // 重投影纠正pnp的yaw

//#define Rune //打符子程序总开关
//#define Auto_Fire_Rune //打符自动开火

#endif
