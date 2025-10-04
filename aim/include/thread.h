#ifndef _THREAD_H_
#define _THREAD_H_
#include "../../control/define.h"
#include "constant.h"
#include "../../drivers/DaHeng/DaHengCamera.h"
#include "predict.h"
#include "serial.h"
#include "../../drivers/Mindvision/MidCamera.h"
#include "../../drivers/Generic/GenericCamera.h"
#include "../../drivers/HikVision/include/hikvision_camera.h"
#include <mutex>
#include <chrono>
#include<future>
#include "vofa.h"

#ifdef TRT
#include "../../trt/TRTModule.h"
#endif

#ifdef OV
#include "../../ov/base_detector.h"
using namespace boost::asio;
#endif


using namespace std;
using namespace Horizon;

enum BufferSize
{
#ifndef NT
    #ifndef Inter_Vis_Ctl
    IMGAE_BUFFER = 10
    #endif
    #ifdef Inter_Vis_Ctl
    IMGAE_BUFFER = 5
    #endif
#endif
#ifdef NT
IMGAE_BUFFER = 5
#endif
}; //lqq ：10 wty：5
    
class Factory
{
public:

    Factory() {
        #ifndef Inter_Vis_Ctl
        #ifndef NT
        coord[0] = 0;
        coord[1] = 0;
        coord[2] = 0;

        rotation[0] = 0;
        rotation[1] = 0;
        rotation[2] = 0;
        #endif
        #endif
        // 读取配置文件
        cv::FileStorage fs("../control/aim_config.yaml", cv::FileStorage::READ);
        fs["outpost_state"] >> outpost_state;

        fs["detect_color"] >> detect_color;
        fs["TRT_confidence"] >> TRT_confidence;
        fs["yaw_max_edge"] >> yaw_max_edge;
        fs["pre_yaw_maxdif"] >> pre_yaw_maxdif;
        //fs["pit_max_edge"] >> yaw_max_edge;
        //fs["pre_pit_maxdif"] >> pre_yaw_maxdif;
        fs["fire_yaw_dif"] >> fire_yaw_dif;
        fs["fire_pit_dif"] >> fire_pit_dif;
        fs["default_fire_v0"] >> default_fire_v0;
        fs["EnginePath"] >> EnginePath;
        fs.release();
    }


public:
    //wty
     cv::Mat image_buffer_[BufferSize::IMGAE_BUFFER];
     double timer_buffer_[IMGAE_BUFFER]; 
    
    //lqq
    //cv::Mat image_buffer_[50];
    //double timer_buffer_[50]; 
    
    volatile unsigned int image_buffer_front_ = 0;   // the produce index
    volatile unsigned int image_buffer_rear_ = 0;    // the comsum index 
    void producer();
    void consumer();
    #ifndef Inter_Vis_Ctl
    void sr_serial();
    #endif
    #ifdef Inter_Vis_Ctl
    //[[noreturn]] void producer();
    //[[noreturn]] void consumer();
    [[noreturn]] void sr_IVC();
    void DebugVofa();
    Stm32Data TimeSynchronization(std::deque<Stm32Data> &stm32s, double src_time);//时钟同步
    #endif

public:
#ifndef NT
    cv::Mat img;
    std::vector<cv::Mat> frames; 
#endif
#ifdef NT
    cv::Mat img{};
    std::vector<cv::Mat> frames{};
#endif

    #ifdef OV
    DetectorProcess infer;
    #endif
    std::shared_ptr<PredictorPose> predic_pose_ = std::make_shared<PredictorPose>(); // 解算器
    std::shared_ptr<PredictorPose> predic_pose_1 = std::make_shared<PredictorPose>(); // 解算器
    std::shared_ptr<PredictorPose> predic_pose_2 = std::make_shared<PredictorPose>(); // 解算器
    std::shared_ptr<PredictorPose> predic_pose_3 = std::make_shared<PredictorPose>(); // 解算器
    //std::shared_ptr<PnpSolver> pnp_solver_ = std::make_shared<PnpSolver>(yaml);//by lqq 整车估计功能组件，尚未完成

    Eigen::Vector3d coord{}; // 世界坐标
    Eigen::Vector3d rotation{};

    mutex serial_mutex_; // 数据上🔓

    GimbalPose imu_data; // 电控发来的数据
    GimbalPose imu_data1;
    int fd;

    #ifndef Inter_Vis_Ctl
    Horizon::DataControler data_controler_;
    Horizon::DataControler::VisionData visiondata; // 视觉向电控传数据
    Horizon::DataControler::Stm32Data stm32data;   // 电控向视觉发数据
    Horizon::DataControler::Stm32Data last_stm32_;
    Horizon::DataControler::Stm32Data stm32data_temp;
    #endif
    
    // 定义发数的位姿
    #ifndef NT
    GimbalPose gim_cur; 
    GimbalPose gim_pre;
    GimbalPose gim_raw_cur;
    GimbalPose gim_raw_pre;
    #endif
    #ifdef NT
    GimbalPose gim;                  // 定义发数的位姿
    #endif

    //aim_config.yaml所控制的信息
    string EnginePath;               // 模型路径
    bool outpost_state{};              // 前哨站状态
    int detect_color;
    float TRT_confidence{}; //置信度
    float TRT_nms_threshold{}; //非极大值抑制
    float yaw_max_edge{};
    float pre_yaw_maxdif{};
    float fire_yaw_dif{};
    float fire_pit_dif{};
    float default_fire_v0{}; // 发射速度

    #ifdef Time_Sync_Serial 
    bool is_aim_;
    Horizon::DataControler::Stm32Data TimeSynchronization(std::deque<Horizon::DataControler::Stm32Data> &stm32s, double src_time);
    CircularQueue<Horizon::DataControler::Stm32Data, 1000> stm32_deque_;
    std::deque<Horizon::DataControler::Stm32Data> MCU_data_;
    int mcu_size_ = 200;
    #endif
    
};
#endif

