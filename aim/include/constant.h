#ifndef CONSTANT_H
#define CONSTANT_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sys/time.h>
#include <thread>
#include <termio.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <sstream>
#include <numeric>
#include <chrono>
#include <vector>
#include <dirent.h>
//来自lsn omni
#include <sched.h> 
#include <ctime> 
//来自（rm_auto）& lsn omni
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
//#include <arpa/inet.h>
#include <unistd.h>
#include "../../control/define.h"

#ifdef TRT
#include "NvInfer.h"
#include "cuda_runtime_api.h"
#include "../../trt/logging.h"
#include "../../trt/preprocess.h"
#include "../../trt/macros.h"
#include "cuda_runtime.h"
#include "NvOnnxParser.h"
using namespace nvonnxparser;
using namespace nvinfer1;

#endif

#ifdef OV
#include "openvino/openvino.hpp"
#include "openvino/opsets/opset9.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#endif

using namespace std;
using namespace cv;
using namespace Eigen;


#define CHECK(status)                                          \
    do                                                         \
    {                                                          \
        auto ret = (status);                                   \
        if (ret != 0)                                          \
        {                                                      \
            std::cerr << "Cuda failure: " << ret << std::endl; \
            abort();                                           \
        }                                                      \
    } while (0)

#define MAX_IMAGE_INPUT_SIZE_THRESH 10000 * 10000
#define MAX_OUTPUT_BBOX_COUNT 1000

#define PROCESSOR_1_MASK                    0x01 << 1
#define PROCESSOR_2_MASK                    0x01 << 2
#define PROCESSOR_3_MASK                    0x01 << 3

// 返回当前时间
static long now()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
/*
    * @brief:  位姿的实现方式
    *
    */
class GimbalPose
{
public:
    float pitch;
    float yaw;
    float roll;
    double timestamp;
    // 初始化函数
    GimbalPose(float pitch = 0.0, float yaw = 0.0, float roll = 0.0)
    {
        this->pitch = pitch;
        this->yaw = yaw;
        this->roll = roll;
    }
    // 左值
    GimbalPose operator=(const GimbalPose &gm)
    {
        this->pitch = gm.pitch;
        this->yaw = gm.yaw;
        this->roll = gm.roll;
        this->timestamp = gm.timestamp;
        return *this;
    }
    GimbalPose operator=(const float init_value)
    {
        this->pitch = init_value;
        this->yaw = init_value;
        this->roll = init_value;
        this->timestamp = now();
        return *this;
    }
    friend GimbalPose operator-(const GimbalPose &gm1, const GimbalPose gm2)
    {
        GimbalPose temp{};
        temp.pitch = gm1.pitch - gm2.pitch;
        temp.yaw = gm1.yaw - gm2.yaw;
        temp.roll = gm1.roll - gm2.roll;
        temp.timestamp = now();
        return temp;
    }
    friend GimbalPose operator+(const GimbalPose &gm1, const GimbalPose gm2)
    {
        GimbalPose temp{};
        temp.pitch = gm1.pitch + gm2.pitch;
        temp.yaw = gm1.yaw + gm2.yaw;
        temp.roll = gm1.roll + gm2.roll;
        temp.timestamp = now();
        return temp;
    }
    friend GimbalPose operator*(const GimbalPose &gm, const float k)
    {
        GimbalPose temp{};
        temp.pitch = gm.pitch * k;
        temp.yaw = gm.yaw * k;
        temp.roll = gm.roll * k;
        temp.timestamp = now();
        return temp;
    }
    friend GimbalPose operator*(const float k, const GimbalPose &gm)
    {
        GimbalPose temp{};
        temp.pitch = gm.pitch * k;
        temp.yaw = gm.yaw * k;
        temp.roll = gm.roll * k;
        temp.timestamp = now();
        return temp;
    }
    friend GimbalPose operator/(const GimbalPose &gm, const float k)
    {
        GimbalPose temp{};
        temp.pitch = gm.pitch / k;
        temp.yaw = gm.yaw / k;
        temp.roll = gm.roll / k;
        temp.timestamp = now();
        return temp;
    }
    friend std::ostream &operator<<(std::ostream &out, const GimbalPose &gm)
    {
        out << "[pitch : " << gm.pitch << ", yaw : " << gm.yaw << "]" << endl;
        return out;
    }

}; 
//}

#ifndef Inter_Vis_Ctl
template<class T,int BUFFER>
class CircularQueue {
public:
    CircularQueue()
    {
        init();
        std::cout << "[enter queue]" << std::endl;
    }

    bool Enqueue(T &val);

    void init();

    int size();

private:
    // const static int BUFFER = 60;
    long long int iter_;
    int size_;
public:
    T queue_[BUFFER];
};

template <class T,int BUFFER>
bool CircularQueue<T,BUFFER>::Enqueue(T &val)
{
    queue_[iter_%BUFFER] = val;
    iter_++;
    size_ = iter_;
    if(iter_ >= BUFFER)
    {
        size_ = BUFFER;
    }
}

template <class T,int BUFFER>
void CircularQueue<T,BUFFER>::init()
{
    iter_ = 0;
    size_ = 0;
}

template <class T,int BUFFER>
int CircularQueue<T,BUFFER>::size()
{
    return size_;
}
#endif

#ifdef Inter_Vis_Ctl
namespace Horizon
{
class Speed
{
public:
    Speed() : rate(0.0), direction(0) {}
    double rate;   // 速率
    int direction; // 方向，-1为逆时针，0为不转，1为顺时针转
};
}
#endif

#endif
