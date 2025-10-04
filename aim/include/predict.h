#ifndef _PREDICT_H_
#define _PREDICT_H_

#include "../../control/define.h"
#include "constant.h"
#include <ceres/jet.h>
#include <ceres/ceres.h>
#include <iostream>
#include <algorithm>
#include <opencv4/opencv2/core/persistence.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include "kf.h"
#include "vofa.h"
// #include <matplotlibcpp.h>

#ifdef TRT
#include "../../trt/TRTModule.h"
using namespace TRTInferV1;
#endif

#ifdef OV
#include "../../ov/base_detector.h"
#endif

//终端带颜色文字输出颜色代号
#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define REDCOLOR "\033[31m"               /* Red */
#define GREENCOLOR "\033[32m"              /* Green */
#define YELLOWCOLOR "\033[33m"             /* Yellow */
#define BLUECOLOR "\033[34m"              /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITECOLOR "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

// namespace plt = matplotlibcpp;


//旧版大小装甲板大小设定方法（现已使用yaml控制）
// 小装甲实际大小
// static const float kRealSmallArmorWidth = 13.5;
// static const float kRealSmallArmorHeight = 5.7;
// 大装甲实际大小
// static const float kRealLargeArmorWidth = 22.5;
// static const float kRealLargeArmorHeight = 5.7;


using namespace Eigen;
#ifdef Inter_Vis_Ctl
using namespace Horizon;
#endif

static ceres::Jet<double, 1> uv_repreject[8];

class PnpSolver
{
public:
    #ifndef NT
    PnpSolver() = delete; // 删除默认构造函数
    PnpSolver(const string yaml);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> poseCalculation(DetectObject &obj,bool t_flag);
    #endif
    #ifdef NT
    PnpSolver(const string yaml);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> poseCalculation(DetectObject &obj);
    #endif

public:
    cv::Mat K_;          // 内参
    cv::Mat distCoeffs_; // 畸变系数
public:
    cv::Mat rotate_world_cam_; // 从世界系到相机系的旋转矩阵
    bool is_large_{}; // 是否是大装甲板

    //yaml控制装甲板大小
    float kRealSmallArmorWidth{}, kRealSmallArmorHeight{};
    float kRealLargeArmorWidth{}, kRealLargeArmorHeight{};

};

#ifndef NT
enum class ARMOR_STATE_
{
    LOSS = 0,
    TRACK = 1
};
#endif
#ifdef NT
enum class ARMOR_STATE_
{
    LOSS = 0,
    INIT = 1,
    TEMP_LOSS = 2,
    TRACK = 3,
    GRYO = 4
};
#endif

struct Predict
{
    /*
     * 此处定义匀速直线运动模型
     */
    template <class T>
    void operator()(const T *x0, T *x1)
    {                                    // x0[6],x1[6]
        x1[0] = x0[0] + delta_t * x0[1]; // 0.1, x
        x1[1] = x0[1];                   // 100
        x1[2] = x0[2] + delta_t * x0[3]; // 0.1, y
        x1[3] = x0[3];                   // 100
        x1[4] = x0[4] + delta_t * x0[5]; // 0.1, z
        x1[5] = x0[5];                   // 100
        #ifdef NT
        //NT
        x1[6] = x0[6] + delta_t * x0[7]; // 旋转角度=角速度*时间
        x1[7] = x0[7];
        x1[8] = x0[8];
        #endif
    }

    double delta_t;
};

template <class T>
void xyz2pyd(T *xyz, T *pyd) // xyz[3], pyd[3]
{
    /*
     * 工具函数：将 xyz 转化为 pitch、yaw、distance
     */
    pyd[0] = ceres::atan2(xyz[1], ceres::sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2])); // pitch
    pyd[1] = ceres::atan2(xyz[0], xyz[2]);                                         // yaw
    pyd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);     // distance
}

#ifndef NT
struct Measure
{
    /*
     * 工具函数的类封装
     */
    template <class T>
    void operator()(const T *x, T *y)
    { // x[6], y[3]
        T x_[3] = {x[0], x[2], x[4]};
        xyz2pyd(x_, y);
    }
};
#endif
#ifdef NT
struct Measure
{
    /*
     * 工具函数的类封装
     */
    template <class T>
    void operator()(const T *x, T *y)
    { // x[6], y[3]     x=x+cosx  整车预测模型 xc yc zc vx vy vz yaw  yaw_v  r
        T XYZ_arrmor[3] = {x[0] + ceres::cos(x[6]) * x[8],
                           x[2] + ceres::sin(x[6]) * x[8],
                           x[4]};
        // T x_[3] = {x[0], x[2], x[4]};

        xyz2pyd(XYZ_arrmor, y);
        y[3] = x[6];
    }
};
#endif

/**
 * @brief  自适应扩展卡尔曼滤波
 *
 * @author 上交:唐欣阳(花山甲老师yyds)
 */

template <int N_X, int N_Y>
class AdaptiveEKF
{
    #ifndef NT
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
    using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;

public:
    explicit AdaptiveEKF(const VectorX &X0 = VectorX::Zero())
        : Xe(X0), P(MatrixXX::Identity())
    {
        std::cout << P << std::endl;
        Q << 0.15, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.3, 0, 0,
            0, 0, 0, 0, 0.5, 0,
            0, 0, 0, 0, 0, 1;

        R << 4, 0, 0,
            0, 1, 0,
            0, 0, 0.5;
    }
    #endif
    #ifdef NT
    //NT
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
    using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;

public:
    explicit AdaptiveEKF(const VectorX &X0 = VectorX::Zero())
        : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity()), R(MatrixYY::Identity())
    {
        // std::cout << P << std::endl;
        cv::FileStorage debug("../control/aim_config.yaml", cv::FileStorage::READ);
        //cv::FileStorage debug(debug_yaml, cv::FileStorage::READ);
        debug["q_x_x"] >> q_x_x;
        debug["q_x_vx"] >> q_x_vx;
        debug["q_vx_vx"] >> q_vx_vx;
        debug["q_y_y"] >> q_y_y;
        debug["q_y_vy"] >> q_y_vy;
        debug["q_vy_vy"] >> q_vy_vy;
        debug["q_r"] >> q_r;
        debug["r_yaw"] >> r_yaw;
        debug["x1"] >> x1;
        debug["x2"] >> x2;
        debug["x3"] >> x3;
        debug.release();
        //   xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        Q << q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0,
            q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0,
            0, 0, q_x_x, q_x_vx, 0, 0, 0, 0, 0,
            0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0,
            0, 0, 0, 0, q_x_x, q_x_vx, 0, 0, 0,
            0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0,
            0, 0, 0, 0, 0, 0, q_y_y, q_y_vy, 0,
            0, 0, 0, 0, 0, 0, q_y_vy, q_vy_vy, 0,
            0, 0, 0, 0, 0, 0, 0, 0, q_r;

        R = Eigen::Matrix<double, N_Y, N_Y>::Zero();
        R.diagonal() << x1, x2, x3, r_yaw;
    }
    #endif
    void init(const VectorX &X0 = VectorX::Zero())
    {
        Xe = X0;
    }

    template <class Func>
    VectorX predict(Func &&func)
    {
        ceres::Jet<double, N_X> Xe_auto_jet[N_X];

        for (int i = 0; i < N_X; i++)
        {
            Xe_auto_jet[i].a = Xe[i];
            Xe_auto_jet[i].v[i] = 1;
        }

        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        func(Xe_auto_jet, Xp_auto_jet);
        for (int i = 0; i < N_X; i++)
        {
            Xp[i] = Xp_auto_jet[i].a;
            F.block(i, 0, 1, N_X) = Xp_auto_jet[i].v.transpose();
        }
        std::cout << F * P * F.transpose() << std::endl;
        P = F * P * F.transpose() + Q;
        std::cout << "predict variables is x: " << Xp[0] << " y: " << Xp[2] << " z: " << Xp[4] << std::endl;

        return Xp;
    }

    template <class Func>
    VectorX update(Func &&func, const VectorY &Y)
    {
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        for (int i = 0; i < N_X; i++)
        {
            Xp_auto_jet[i].a = Xp[i];
            Xp_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Yp_auto_jet[N_Y];
        func(Xp_auto_jet, Yp_auto_jet);
        for (int i = 0; i < N_Y; i++)
        {
            Yp[i] = Yp_auto_jet[i].a;
            H.block(i, 0, 1, N_X) = Yp_auto_jet[i].v.transpose();
        }
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        Xe = Xp + K * (Y - Yp);
        P = (MatrixXX::Identity() - K * H) * P;

        std::cout << "update variables is x: " << Xe[0] << " y: " << Xe[2] << " z: " << Xe[4] << std::endl;
        return Xe;
    }

    VectorX Xe; // 估计状态变量
    VectorX Xp; // 预测状态变量
    MatrixXX F; // 预测雅克比
    MatrixYX H; // 观测雅克比
    MatrixXX P; // 状态协方差
    MatrixXX Q; // 预测过程协方差
    MatrixYY R; // 观测过程协方差
    MatrixXY K; // 卡尔曼增益
    VectorY Yp; // 预测观测量

    #ifdef NT
    //NT
    double q_x_x = 3.2768000000000003e-07;
    double q_x_vx = 4.096e-05;
    double q_vx_vx = 0.0051199999999999996;

    double q_y_y = 1.6384000000000003e-06;
    double q_y_vy = 8.096e-05;
    double q_vy_vy = 0.0095599999999999998;

    double q_r = 1.3107200000000002e-05;

    double r_yaw = 0.45;
    double x1 = abs(0.0025);
    double x2 = abs(0.0025);
    double x3 = abs(0.04);
    #endif

};

#ifdef MIDVISION
const static string yaml = "../drivers/Distortions/camera_info_MD.yaml";
#endif

#ifdef HK
const static string yaml = "../drivers/Distortions/camera_info_HK.yaml";
#endif

#ifdef DAHENG1
const static string yaml = "../drivers/Distortions/camera_info_DH1.yaml";
#endif

#ifdef DAHENG2
const static string yaml = "../drivers/Distortions/camera_info_DH2.yaml";
#endif

#ifdef GENERIC
const static string yaml = "../drivers/Distortions/camera_info_MD.yaml";
#endif
#ifdef VIDEO
const static string yaml = "../drivers/Distortions/camera_info_MD.yaml";
#endif


class PredictorPose
{
public:
        int velocities_deque_size_;
        double X_BIAS, Y_BIAS, Z_BIAS;
        double pit_corangle,yaw_corangle;
        int loss_cnt_max;
        int pre_los_maxq;
        int cru_los_maxq;
        int NT_LOSS_MAXQ; 
        int NT_TRACK_MAXQ;
            
	PredictorPose()
	{
        #ifndef NT
		init_ = true; //NT
		loss_cnt_ = 0;
        #endif
        cv::FileStorage fs("../control/aim_config.yaml", cv::FileStorage::READ);
        fs["X_BIAS"] >> X_BIAS;
        fs["Y_BIAS"] >> Y_BIAS;
        fs["Z_BIAS"] >> Z_BIAS;
        fs["velocities_deque_size_"] >> velocities_deque_size_;
        fs["pit_corangle"] >> pit_corangle;
	    fs["yaw_corangle"] >> yaw_corangle;
	    fs["loss_cnt_max"] >> loss_cnt_max;  
	    fs["pre_los_maxq"] >> pre_los_maxq;   
        fs["cru_los_maxq"] >> cru_los_maxq;  
        fs["NT_LOSS_MAXQ"] >> NT_LOSS_MAXQ;  
        fs["NT_TRACK_MAXQ"] >> NT_TRACK_MAXQ;            
        fs.release();
	};
    
private:
	// 相机系转云台系，转到IMU上没多大作用
	// 转出来的分别是装甲板相对于云台的位置和姿态
	// cam_coord是相机坐标系坐标，rotate_world_cam是世界坐标系到相机坐标系旋转
    #ifndef NT
	std::pair<Eigen::Vector3d, Eigen::Vector3d> cam2ptz(Eigen::Vector3d &cam_coord, cv::Mat &rotate_world_cam);
    #endif
    #ifdef NT
    std::pair<Eigen::Vector3d, Eigen::Vector3d> cam2ptz(std::pair<Eigen::Vector3d, Eigen::Vector3d> &cam_coord, GimbalPose gm);
    #endif

public:
#ifndef NT
	GimbalPose run_predict(GimbalPose &imu_data, std::vector<DetectObject> &objects, double time);
    GimbalPose run_current(GimbalPose &imu_data, std::vector<DetectObject> &objects, double time);
#endif
#ifdef NT
    //NT
    GimbalPose run(GimbalPose &imu_data, std::vector<DetectObject> &objects, double time);

    GimbalPose init(std::vector<DetectObject> &objects, GimbalPose imu_data);

    GimbalPose Track(const std::vector<TRTInferV1::DetectObject> &objects, double current_time, GimbalPose imu_data);

    GimbalPose Loss();

    GimbalPose TEMP_LOSS(double current_time);

    GimbalPose AntiGryo();
#endif

private:
#ifndef NT
	void init()
	{
		init_ = true;
		loss_cnt_ = 0;
		last_state_ = ARMOR_STATE_::LOSS;
	};
#endif
public:
    #ifndef NT
    ARMOR_STATE_ state_ = ARMOR_STATE_::LOSS;  // 装甲板识别状态
	ARMOR_STATE_ last_state_; // 装甲板上一帧状态
	GimbalPose imu_data_; // 云台姿态
    #endif
    #ifdef NT
    ARMOR_STATE_ state = ARMOR_STATE_::LOSS; // 装甲板识别状态
    ARMOR_STATE_ last_state_{};              // 装甲板上一帧状态 NT
    GimbalPose imu_data_{};                  // 云台姿态 NT
    GimbalPose last_eular_{};                // 上一次返回的云台姿态 NT
    GimbalPose gm_ptz;                       // 由世界坐标得到的云台姿态
    #endif

#ifndef NT
/// @brief 状态记录变量
private:
	int loss_cnt_; // 装甲板丢失计数器，如果超过10次，记为丢失，需要初始化
	bool init_;	// 初始化开关: 触发初始化条件，装甲板切换，第一次有装甲板进入预测器，需要初始化为true
#endif
#ifdef NT
    /// 状态记录变量
private:
    int brief_loss_cnt = 0;            // 装甲板丢失计数器
    int track_cnt = 0;                 // 跟踪计数器
    // const int MAX_BRIEF_LOSS_CNT = 35; // 最大丢失次数，现已由NT_LOSS_MAXQ控制
    // const int MAX_TRACK_CNT = 5;       // 最大跟踪次数，现已由NT_TRACK_MAXQ控制
    bool is_gyro_ = false;             // 是否是陀螺状态
#endif

public:
    #ifdef NT
    bool Need_better_target(const std::vector<TRTInferV1::DetectObject> &objects, TRTInferV1::DetectObject last_obj_);
	#endif
    DetectObject ArmorSelect(std::vector<DetectObject> &object,bool t_flag);
    DetectObject ArmorSelect(std::vector<DetectObject> objects);
    DetectObject last_obj_; // 初始化状态下得到的装甲板位置为跟踪状态做准备
    //DetectObject ArmorChoice(std::vector<DetectObject> &objects); //by WTY（未测试）
	
public:
	std::shared_ptr<PnpSolver> pnp_solve_ = std::make_shared<PnpSolver>(yaml); // 解算器
	std::pair<Eigen::Vector3d, Eigen::Vector3d>	last_pose_;
	std::deque<Eigen::Vector4d> velocities_; // 速度的循环队列，方便做拟合，装甲板切换初始化
	Eigen::Vector3d last_velocity_; // 上一时刻的速度
	Eigen::Vector3d last_location_; // 上一时刻目标在云台系下的坐标
	Eigen::Vector3d CeresVelocity(std::deque<Eigen::Vector4d> velocities); // 最小二乘法拟合速度

    Eigen::Matrix3d transform_vector_;
    Eigen::Vector3d predict_location_;
    Eigen::Vector3d current_location_;


#ifndef NT
public:
    #ifndef Inter_Vis_Ctl
	float v0{};	// 弹速
    #endif
    #ifdef Inter_Vis_Ctl
    float v0 = 23;	// 弹速
    #endif
	float bullteFlyTime(Eigen::Vector3d coord); //飞行时间和弹道解算
    float ballistic_equation(Eigen::Vector3d coord); //by wty（未调用，待测试）
	GimbalPose gm_ptz;	// 角度制
	double last_time_;
	double current_time_;
    cv::Point2f obj_pixe_;
public:
    float move_ = 0;
    GimbalPose last_eular_;

public:
    std::deque<Eigen::Vector4d> yaw_rad_; // 投入4个，但是目前只要1个
    int yaw_rad_size_ = 30;
    bool is_gyro_ = false;
    Eigen::Vector3d last_posture_;
    double threshold_yaw_rad_ = 60;
    double R_ = 0.6;

//**************逻辑判断***************//
    int switch_count_ = 0;
    int last_obj_nums = 0;
    float last_id;
    DetectObject last_obj;
    bool switch_flag = false;
    int v_count = 0;
//**************逻辑判断***************//

//**************反陀螺1 开始***************//
    Kalman_Filter kf;
    Vector3d last_yaw_v;
//**************反陀螺1 结束***************//

//**************反陀螺2 开始***************//
    int cnt_switch = 0;
    Vector3d left_switch_;
    Vector3d right_switch_;
    std::deque<double> time_buff_;
    float last_switch_time_;
    int is_gyro_cnt_ = 0;
    bool isSwitch(Vector3d up_switch, Vector3d down_switch);
    
    //bool isGyro(std::deque<double> time_buff_, Vector3d up_switch, Vector3d down_switch); by lqq 未启用

    float time_diff = 0;
//**************反陀螺2 结束***************//

    Eigen::Vector3d cam3ptz(GimbalPose gm,Vector3d pos,bool t_flag);

    //********** 打前哨站 **********//by wty（待测试）
    //void hit_outpost();
    bool find_switch = false;
    bool is_fire = false;
    Eigen::Vector3d mid_pt_ = Eigen::Vector3d(0,0,1);
    #endif
    #ifdef NT

public:
    // EKF
    double r = 0.2;        // 装甲板半径
    double yaw = 0;        // 装甲板旋转角度
    double yaw_v = 0;      // 装甲板角速度
    double last_yaw{};     // 上一时刻装甲板旋转角度
    AdaptiveEKF<9, 4> ekf; // 创建ekf滤波器

    float v0{};                                  // 弹速
    double bullteFlyTime(Eigen::Vector3d coord); // NT飞行时间和弹道解算

    int loss_cnt_ = 0;
    bool init_ = false;
    // 原反陀螺相关
    double last_time_{};
    double current_time_{};
    double last_switch_time_{};
    double time_diff = 0;
    DetectObject last_obj;
    cv::Point2f obj_pixe_;
    bool left_flag = false; // 装甲板是否切换的标志
    bool right_flag = false;
    int cnt_switch = 0;

    Vector3d left_switch;
    Vector3d right_switch;
    bool flag_switch = false;
    int is_gyro_cnt_{};
    std::deque<double> time_buff_;

    bool isSwitch(Vector3d up_switch, Vector3d down_switch);

    bool anGyro(std::deque<double> time_buff_);

    double get_time(std::deque<double> time_buff_);

    // 最小化二乘法拟合速度
    Eigen::Vector3d ema_velocity_ = Eigen::Vector3d::Zero(); // 初始化为0
    const double smoothing_factor = 0.9;                     // 设置平滑系数，范围在0到1之间，值越接近1表示历史数据影响越大
    int v_count = 0;

    // 计算云台系下四个点的坐标
    std::vector<Eigen::Vector3d>
    calculateTargetPoints(const Eigen::Vector3d &center, double width, double height, double yaw);

    // 将四个点重投影到图像坐标系
    std::vector<cv::Point2f> projectPoints(const std::vector<Eigen::Vector3d> &points3d, const cv::Mat &cameraIntrinsics,
                                           const Eigen::Vector3d &turretToCamTranslation);

    // 代价函数
    double objectiveFunc(double yaw, const Eigen::Vector3d &center, double width, double height,
                         const std::vector<cv::Point2f> &boundaryPoints, const cv::Mat &cameraIntrinsics,
                         const Eigen::Vector3d &turretToCamTranslation);

    // 优选法三分法
    double goldenSectionSearch(double a, double b, const Eigen::Vector3d &center, double width, double height,
                               const std::vector<cv::Point2f> &boundaryPoints, const cv::Mat &cameraIntrinsics,
                               const Eigen::Vector3d &turretToCamTranslation);


    double best_yaw(TRTInferV1::DetectObject obj); // 重投影纠正pnp的yaw
    #endif
#ifdef NEXT_Gyro
public:
    // 在现有成员变量后新增
    std::deque<Eigen::Vector3d> mid_pt_buffer_;  // 存储中间点队列
    Eigen::Vector3d ave_mid_pt_;                 // 平均中间点
    bool is_gyro_locked_ = false;                // 是否已锁定中心
    const int GYRO_SAMPLE_NUM = 4;               // 采样次数
    const double GYRO_FIRE_THRESH = 0.15;        // 开火距离阈值(米)
    float calculateYawDiff(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2);
    float calculateDistance(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2);
    // ---
#endif
};

#endif
