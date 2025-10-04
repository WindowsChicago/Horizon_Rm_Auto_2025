#include "../include/thread.h"
#include <thread>
int main()
{
    Factory vision;

    std::thread thread1(&Factory::producer,std::ref(vision));
	
    std::thread thread2(&Factory::consumer,std::ref(vision));
    #ifndef Inter_Vis_Ctl
    std::thread thread3(&Factory::sr_serial,std::ref(vision));
    #endif
    #ifdef Inter_Vis_Ctl
    std::thread thread3(&Factory::sr_IVC,std::ref(vision));
    #endif

    #ifdef CPU_Thread_ID_Lock
     // 绑定第一个线程到指定的 CPU 核心（例如绑定到第一个核心，即CPU0）
    cpu_set_t cpuset1;
    CPU_ZERO(&cpuset1);
    CPU_SET(3, &cpuset1);  // 将第一个核心（CPU3）添加到集合中
    int result1 = pthread_setaffinity_np(thread1.native_handle(), sizeof(cpu_set_t), &cpuset1);
    if (result1 != 0) {
        std::cerr << "无法将第一个线程绑定到指定的 CPU 核心。错误码：" << result1 << std::endl;
        return 1;
    }
    //绑定第二个线程到指定的 CPU 核心（例如绑定到第二个核心，即CPU1）
    cpu_set_t cpuset2;
    CPU_ZERO(&cpuset2);
    CPU_SET(4, &cpuset2);  // 将第二个核心（CPU1）添加到集合中
    int result2 = pthread_setaffinity_np(thread2.native_handle(), sizeof(cpu_set_t), &cpuset2);
    if (result2 != 0) {
        std::cerr << "无法将第二个线程绑定到指定的 CPU 核心。错误码：" << result2 << std::endl;
        return 1;
    }
    // 绑定第三个线程到指定的 CPU 核心（例如绑定到第三个核心，即CPU2）
    cpu_set_t cpuset3;
    CPU_ZERO(&cpuset3);
    CPU_SET(5, &cpuset3);  // 将第二个核心（CPU1）添加到集合中
    int result3 = pthread_setaffinity_np(thread3.native_handle(), sizeof(cpu_set_t), &cpuset3);
    if (result3 != 0) {
        std::cerr << "无法将第3个线程绑定到指定的 CPU 核心。错误码：" << result3 << std::endl;
        return 1;
    }
    #endif
#ifdef Debug_Vofa
    std::thread thread4(&Factory::DebugVofa, std::ref(vision));
    thread4.join();
#endif
    
    thread1.join();

    thread2.join();

    thread3.join();

}

