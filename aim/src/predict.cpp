#include "../include/predict.h"
#include "predict.h"

#ifdef NT
#define SIN_POINT_NUM 400
float SavePoint[SIN_POINT_NUM];    // 保存点位
float SecSavePoint[SIN_POINT_NUM]; // 保存点位
int Times = 0;
#endif

/**
 * @brief PnpSolver类的构造函数，用于初始化相机内参和畸变系数。
 * 
 * 该构造函数通过读取指定的YAML配置文件，加载相机的内参矩阵和畸变系数。
 * 
 * @param yaml YAML配置文件的路径，包含相机的内参矩阵和畸变系数。
 */
PnpSolver::PnpSolver(const string yaml)
{
    // 打开YAML文件并准备读取数据
    cv::FileStorage cam(yaml, cv::FileStorage::READ);
    // 从YAML文件中读取相机内参矩阵并存储到K_中
    cam["M1"] >> K_;
    // 从YAML文件中读取畸变系数并存储到distCoeffs_中
    cam["D1"] >> distCoeffs_;
    // 释放文件资源
    cam.release();
    //yaml控制装甲板大小
	cv::FileStorage debug("../control/aim_config.yaml", cv::FileStorage::READ);
    debug["small_armor"]["width"] >> kRealSmallArmorWidth;
    debug["small_armor"]["height"] >> kRealSmallArmorHeight;
    debug["large_armor"]["width"] >> kRealLargeArmorWidth;
    debug["large_armor"]["height"] >> kRealLargeArmorHeight;
	debug.release();
}

/**
 * @brief 由Pnp得出的旋转向量得到欧拉角
 * 
 * 该函数将输入的旋转向量转换为欧拉角（roll, pitch, yaw）。欧拉角表示绕固定轴的旋转角度，
 * 通常用于描述物体的姿态。
 * 
 * @param rotation_vector 输入的旋转向量，通常是通过OpenCV的旋转向量表示法得到的。
 * @return Eigen::Vector3d 返回一个包含三个欧拉角的向量，依次为 roll, pitch, yaw。
 */
Eigen::Vector3d get_euler_angle(cv::Mat rotation_vector)
{
    // 计算旋转向量的模长，即旋转角度 theta
    double theta = cv::norm(rotation_vector, cv::NORM_L2);

    // 将旋转向量转换为四元数表示
    double w = std::cos(theta / 2);
    double x = std::sin(theta / 2) * rotation_vector.ptr<double>(0)[0] / theta;
    double y = std::sin(theta / 2) * rotation_vector.ptr<double>(1)[0] / theta;
    double z = std::sin(theta / 2) * rotation_vector.ptr<double>(2)[0] / theta;

    double ysqr = y * y;

    // 计算 pitch (绕x轴的旋转)
    double t0 = 2.0 * (w * x + y * z);
    double t1 = 1.0 - 2.0 * (x * x + ysqr);
    double pitch = std::atan2(t0, t1);

    // 计算 yaw (绕y轴的旋转)
    double t2 = 2.0 * (w * y - z * x);
    if (t2 > 1.0)
    {
        t2 = 1.0;
    }
    if (t2 < -1.0)
    {
        t2 = -1.0;
    }
    double yaw = std::asin(t2);

    // 计算 roll (绕z轴的旋转)
    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (ysqr + z * z);
    double roll = std::atan2(t3, t4);

    // 返回欧拉角，顺序为 roll, pitch, yaw
    return {roll, pitch, yaw};
}

#ifndef NT
/**
 * @brief:  位姿解算器
 *
 * @author: liqianqi
 *
 * @param:  obj: 装甲板信息，主要用四点
 *
 * @return: 装甲板在相机系的位置和姿态
 */
//桥接trt模块和预测模块的函数
std::pair<Eigen::Vector3d, Eigen::Vector3d> PnpSolver::poseCalculation(DetectObject &obj,bool t_flag)
{
	if(t_flag == true)
	{
	std::cout << "=======================" << std::endl;
    std::cout << "[pose_solver] 位姿解算" << std::endl;
	}
    std::vector<cv::Point3f> point_in_world; // 装甲板世界坐标系

    #ifdef TRT
    // 检查目标物体的像素点数量是否为4，如果不是则进行修正
    if(obj.pts.size() != 4)
    {
		if(t_flag == true)
		{
        std::cout << "enter pts error!!!" << obj.pts.size() << std::endl;
		}
        cv::Point2d uv[4];
        for(int i = 0; i < 4; i++)
        {
            uv[i].x = obj.pts[i].x;
            uv[i].y = obj.pts[i].y;
        }
        obj.pts.clear();
        for(int i = 0; i < 4; i++)
        {
            obj.pts.emplace_back(uv[i]);
        }
    }
    #endif

    // 计算目标物体的宽高比
    float width_height_ratio = std::sqrt(std::pow(obj.pts[3].x - obj.pts[0].x, 2) + std::pow(obj.pts[3].y - obj.pts[0].y, 2)) / std::sqrt(std::pow(obj.pts[1].x - obj.pts[0].x, 2) + std::pow(obj.pts[1].y - obj.pts[0].y, 2));
	if(t_flag == true)
	{
    std::cout << "=======================" << std::endl;
	}
    std::vector<cv::Point2f> point_in_pixe; // 像素坐标系

    // 将目标物体的像素坐标存入point_in_pixe
    for (auto pt : obj.pts)
    {
        point_in_pixe.emplace_back(pt);
    }
    float id = obj.label;
	// point_in_pixe.push_back(obj.pts[0]);
	// point_in_pixe.push_back(obj.pts[1]);2
	// point_in_pixe.push_back(obj.pts[2]);
	// point_in_pixe.push_back(obj.pts[3]);

    // 打印目标物体的像素坐标
	if(t_flag == true)
	{	
    std::cout << "[x,y]  " << point_in_pixe[0].x << " " << point_in_pixe[0].y << std::endl;
    std::cout << "[x,y]  " << point_in_pixe[1].x << " " << point_in_pixe[1].y << std::endl;
    std::cout << "[x,y]  " << point_in_pixe[2].x << " " << point_in_pixe[2].y << std::endl;
    std::cout << "[x,y]  " << point_in_pixe[3].x << " " << point_in_pixe[3].y << std::endl;
	}

    // 根据宽高比和目标物体类别，确定目标物体的大小，并设置世界坐标系中的3D坐标
    if (width_height_ratio < 3.0 && (id != 1 && id != 7))
    {
		if(t_flag == true)
		{
        std::cout << "[notice] the small armor" << std::endl;
		}
        is_large_ = false;
        float fHalfX = kRealSmallArmorWidth * 0.5f;	 // 将装甲板的宽的一半作为原点的x
        float fHalfY = kRealSmallArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
        point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
        point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
        point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));
        point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));
    }
    else
    {
		if(t_flag == true)
		{
        std::cout << "[notice] the large armor" << std::endl;
		}
        is_large_ = true;
        float fHalfX = kRealLargeArmorWidth * 0.5f;	 // 将装甲板的宽的一半作为原点的x
        float fHalfY = kRealLargeArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
        point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
        point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
        point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));
        point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));
    }
	// std::cout << "[notice] the small armor" << std::endl;
	// float fHalfX = kRealSmallArmorWidth * 0.5f;	 // 将装甲板的宽的一半作为原点的x
	// float fHalfY = kRealSmallArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
	// point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
	// point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
	// point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));
	// point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));

    // 检查世界坐标系和像素坐标系中的点数量是否为4
    if (point_in_world.size() == 4 && point_in_pixe.size() == 4)
    {
		if(t_flag == true)
		{
        std::cout << "world and pixe all four points" << std::endl;
		}
    }
    else
    {
		if(t_flag == true)
		{
		std::cout << "[world] size " << point_in_world.size() << std::endl;
        std::cout << "[pixe] size " << point_in_pixe.size() << std::endl;
		}
    }

    // 初始化旋转向量和平移向量
    cv::Mat rvecs = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvecs = cv::Mat::zeros(3, 1, CV_64FC1);

	// 世界坐标系到相机坐标系的变换
	// tvecs 表示从相机系到世界系的平移向量并在相机系下的坐标
	// rvecs 表示从相机系到世界系的旋转向量，需要做进一步的转换
	// 默认迭代法: SOLVEPNP_ITERATIVE，最常规的用法，精度较高，速度适中，一次解算一次耗时不到1ms

    // 使用PnP算法计算目标物体的位姿
    cv::solvePnP(point_in_world, point_in_pixe, K_, distCoeffs_, rvecs, tvecs, false, cv::SOLVEPNP_UPNP);

    // 将旋转向量转换为旋转矩阵
    cv::Mat rotM = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Rodrigues(rvecs, rotM);

    rotate_world_cam_ = rotM;

    // 将OpenCV的旋转矩阵转换为Eigen的矩阵
    Eigen::Matrix3d rotM_eigen;
    cv2eigen(rotM, rotM_eigen);

	// 将旋转矩阵分解为三个轴的欧拉角（roll、pitch、yaw）
	// Eigen::Vector3d euler_angles = rotationMatrixToEulerAngles(rotM_eigen);

    // 计算旋转矩阵对应的欧拉角（roll、pitch、yaw）
    Eigen::Vector3d euler_angles = get_euler_angle(rvecs);

    double roll = euler_angles[0];	// X-world-axis 与 X-cam-axis 在yoz上的夹角   roll
    double pitch = euler_angles[1]; // Y-world-axis 与 Y-cam-axis 在xoz上的夹角   yaw
    double yaw = euler_angles[2];	// Z-world-axis 与 Z-cam-axis 在xoy上的夹角   pitch

    // 将平移向量转换为Eigen的向量，并调整单位
    Eigen::Vector3d coord;
    coord << tvecs.ptr<double>(0)[0] / 100, -tvecs.ptr<double>(0)[1] / 100, tvecs.ptr<double>(0)[2] / 100;

    // 将欧拉角存入Eigen的向量
    Eigen::Vector3d rotation;
    rotation << roll, pitch, yaw;

	if(t_flag == true)
	{
    // 打印旋转角度
    std::cout << "[roll] " << roll * 180 / CV_PI << "  "
              << "[pitch] " << pitch * 180 / CV_PI << "  "
              << "[yaw] " << yaw * 180 / CV_PI << std::endl;
	}
    // 将位姿信息存入pair并返回
    std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
    pose.first = coord;
    pose.second = rotation;

    // 打印目标物体的3D坐标
	if(t_flag == true)
	{
    std::cout << "[x]: " << coord[0] << "  "
              << "[y]: " << coord[1] << "  [z]: " << coord[2] << std::endl;

    std::cout << "camera pose finished!!!" << std::endl;
    std::cout << "=======================" << std::endl;
	}
    return pose;
}
/**
 * @brief:  相机系到云台系的姿态
 *
 * @author: liqianqi
 * 该函数通过给定的相机坐标系下的坐标和旋转矩阵，结合IMU数据（俯仰角和偏航角），
 * 将相机坐标系下的坐标转换为云台坐标系下的坐标。同时，计算并返回云台坐标系下的坐标和旋转向量。
 * 
 * @param cam_coord 相机坐标系下的坐标，类型为Eigen::Vector3d
 * @param rotate_world_cam 从世界坐标系到相机坐标系的旋转矩阵，类型为cv::Mat
 * @param:  cam_coord是相机系的坐标, rotate_world_cam是从世界系转相机系的旋转矩阵
 *
 * @return: 装甲板在云台系的位置和姿态
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d> PredictorPose::cam2ptz(Eigen::Vector3d &cam_coord, cv::Mat &rotate_world_cam)
{
    // 对相机坐标系下的坐标进行偏置调整
    cam_coord[0] = cam_coord[0] + X_BIAS;
    cam_coord[1] = cam_coord[1] + Y_BIAS;
    cam_coord[2] = cam_coord[2] + Z_BIAS;

    // 将OpenCV的旋转矩阵转换为Eigen的矩阵格式
    Eigen::Matrix3d rotate_world_cam_eigen;
    cv::cv2eigen(rotate_world_cam, rotate_world_cam_eigen);

    // 根据IMU的俯仰角（pitch）构建俯仰旋转矩阵
    Eigen::Matrix3d pitch_rotation_matrix_t;
    pitch_rotation_matrix_t
        << 1,
        0, 0,
        0, std::cos((imu_data_.pitch * CV_PI) / 180), std::sin((imu_data_.pitch * CV_PI) / 180),
        0, -std::sin((imu_data_.pitch * CV_PI) / 180), std::cos((imu_data_.pitch * CV_PI) / 180);

    // 根据IMU的偏航角（yaw）构建偏航旋转矩阵
	std::cout << "yaw data is " << imu_data_.yaw << std::endl;
    Eigen::Matrix3d yaw_rotation_matrix_t;
    yaw_rotation_matrix_t
        << std::cos(imu_data_.yaw * CV_PI / 180),
        0, std::sin(imu_data_.yaw * CV_PI / 180),
        0, 1, 0,
        -std::sin(imu_data_.yaw * CV_PI / 180), 0, std::cos(imu_data_.yaw * CV_PI / 180);

    // 打印两种旋转矩阵的乘积，用于调试和验证
    std::cout << yaw_rotation_matrix_t * pitch_rotation_matrix_t << std::endl;
    std::cout << pitch_rotation_matrix_t * yaw_rotation_matrix_t << std::endl;
	/**
	 * 写两种矩阵的原因是因为位置和姿态所用坐标系不同
	 * 用欧拉较或泰特布莱恩角表示旋转，顺序十分重要，一般是X-Y-Z，但RM这种小角度，绕定轴动轴都一样顺序什么样结果都一样
	 * 可以动手试试
	 */
	
    // 将相机坐标系下的坐标转换为云台坐标系下的坐标
    Eigen::Vector3d ptz_coord = yaw_rotation_matrix_t * pitch_rotation_matrix_t * cam_coord;

    // 计算变换矩阵并保存其逆矩阵
    Eigen::Matrix3d transform_vector;
    transform_vector = yaw_rotation_matrix_t * pitch_rotation_matrix_t;
    transform_vector_ = transform_vector.inverse();

    // 初始化旋转向量
    Eigen::Vector3d rotation;
    rotation << 0, 0, 0;

    // 返回云台坐标系下的坐标和旋转向量
    std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
    pose.first = ptz_coord;
    pose.second = rotation;

    return pose;
}
#endif
#ifdef NT
/**
 * @brief 判断两个开关位置是否需要进行切换，并根据位置关系确定左右开关的位置。
 * 
 * 该函数通过计算两个开关位置在x轴和z轴上的距离，判断是否需要进行切换。如果距离大于阈值，
 * 则根据x轴上的位置关系确定左右开关的位置。
 * 
 * @param up_switch 第一个开关的位置，类型为Vector3d，包含x、y、z三个坐标。
 * @param down_switch 第二个开关的位置，类型为Vector3d，包含x、y、z三个坐标。
 * @return bool 返回true表示需要进行切换，并已确定左右开关的位置；返回false表示不需要切换。
 */
bool PredictorPose::isSwitch(Vector3d up_switch, Vector3d down_switch) 
{
    // 计算两个开关在x轴和z轴上的绝对距离
    float x1 = std::abs(up_switch[0] - down_switch[0]);
    float z1 = std::abs(up_switch[2] - down_switch[2]);

    // 判断x轴和z轴上的距离之和是否大于阈值0.3
    if (x1 + z1 > 0.3)
    {
        // 根据x轴上的位置关系确定左右开关的位置
        if (up_switch[0] > down_switch[0])
        {
            right_switch = up_switch;
            left_switch = down_switch;
        }
        else
        {
            right_switch = down_switch;
            left_switch = up_switch;
        }
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * @brief:  位姿解算器
 *
 * @author: liqianqi
 *
 * @param:  obj: 装甲板信息，主要用四点
 *
 * @return: 装甲板在相机系的位置和姿态
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d> PnpSolver::poseCalculation(DetectObject &obj)
{
    std::cout << "=======================" << std::endl;
    std::cout << "[pose_solver] poseCalculation" << std::endl;
    std::vector<cv::Point3f> point_in_world; // 装甲板世界坐标系
    float width_height_ratio =
        std::sqrt(std::pow(obj.pts[3].x - obj.pts[0].x, 2) + std::pow(obj.pts[3].y - obj.pts[0].y, 2)) /
        std::sqrt(std::pow(obj.pts[1].x - obj.pts[0].x, 2) + std::pow(obj.pts[1].y - obj.pts[0].y, 2));
    std::vector<cv::Point2f> point_in_pixe; // 像素坐标系

    for (int i = 0; i < 4; i++)
    {
        point_in_pixe.push_back(obj.pts[i]); // 像素坐标系
    }
    for (int i = 0; i < 4; i++)
    {
        std::cout << "\033[31m" << point_in_pixe[i] << "\033[0m" << std::endl;
    }
    if (width_height_ratio < 2.7)
    {
        std::cout << "[notice] the small armor" << std::endl;
        is_large_ = false;
        float fHalfX = kRealSmallArmorWidth * 0.5f;       // 将装甲板的宽的一半作为原点的x
        float fHalfY = kRealSmallArmorHeight * 0.5f;      // 将装甲板的宽的一半作为原点的y
        point_in_world.emplace_back(-fHalfX, fHalfY, 0);  // 2
        point_in_world.emplace_back(-fHalfX, -fHalfY, 0); // 3
        point_in_world.emplace_back(fHalfX, -fHalfY, 0);  // 4
        point_in_world.emplace_back(fHalfX, fHalfY, 0);   // 1
    }
    else
    {
        std::cout << "[notice] the large armor" << std::endl;
        is_large_ = true;
        float fHalfX = kRealLargeArmorWidth * 0.5f;  // 将装甲板的宽的一半作为原点的x
        float fHalfY = kRealLargeArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
        point_in_world.emplace_back(-fHalfX, fHalfY, 0);
        point_in_world.emplace_back(-fHalfX, -fHalfY, 0);
        point_in_world.emplace_back(fHalfX, -fHalfY, 0);
        point_in_world.emplace_back(fHalfX, fHalfY, 0);
    }

    if (point_in_world.size() != 4 && point_in_pixe.size() != 4)
    {
        std::cout << "[world] size " << point_in_world.size() << std::endl;
        std::cout << "[pixe] size " << point_in_pixe.size() << std::endl;
    }

    cv::Mat rvecs = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvecs = cv::Mat::zeros(3, 1, CV_64FC1);

    // 世界坐标系到相机坐标系的变换
    // tvecs 表示从相机系到世界系的平移向量并在相机系下的坐标
    // rvecs 表示从相机系到世界系的旋转向量，需要做进一步的转换
    // 默认迭代法: SOLVEPNP_ITERATIVE，最常规的用法，精度较高，速度适中，一次解算一次耗时不到1ms
    cv::solvePnP(point_in_world, point_in_pixe, K_, distCoeffs_, rvecs, tvecs, cv::SOLVEPNP_ITERATIVE);
    cv::Mat rotM = cv::Mat::zeros(3, 3, CV_64FC1); // 解算出来的旋转矩阵

    cv::Rodrigues(rvecs, rotM);

    rotate_world_cam_ = rotM;
    // 将旋转矩阵分解为三个轴的欧拉角（roll、pitch、yaw）
    Eigen::Vector3d euler_angles = get_euler_angle(rvecs);

    double roll = euler_angles[0];  // X-world-axis 与 X-cam-axis 在yoz上的夹角   roll
    double yaw = euler_angles[1];   // Y-world-axis 与 Y-cam-axis 在xoz上的夹角   yaw
    double pitch = euler_angles[2]; // Z-world-axis 与 Z-cam-axis 在xoy上的夹角   pitch

    Eigen::Vector3d coord;
    coord << tvecs.ptr<double>(0)[0] / 100, -tvecs.ptr<double>(0)[1] / 100, tvecs.ptr<double>(0)[2] / 100;

    Eigen::Vector3d rotation;
    rotation << roll, pitch, yaw;
    // std::cout << "euler_[roll] " << roll * 180 / CV_PI << "  "
    // 		  << "euler_[pitch] " << pitch * 180 / CV_PI << "  "
    // 		  << "euler_[yaw] " << yaw * 180 / CV_PI << std::endl;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
    pose.first = coord;
    pose.second = rotation;

    // std::cout << "[tx]: " << tvecs.ptr<double>(0)[0] / 100 << "  "
    // 		  << "[ty]: " << -tvecs.ptr<double>(0)[1] / 100 << "  [tz]: " << tvecs.ptr<double>(0)[2] / 100 << std::endl;
    toVofa.tz_camera = tvecs.ptr<double>(0)[2] / 100;
    toVofa.yaw_camera = yaw * 180 / CV_PI;
    toVofa.roll_camera = roll * 180 / CV_PI;
    // 为什么要传相机系的旋转角，因为当无法上车调试时，可以将就着调
    std::cout << "camera pose finished!!!" << std::endl;
    std::cout << "=======================" << std::endl;
    return pose;
}
/**
 * @brief:  相机系到云台系的转换
 *
 * @author: liqianqi
 *
 * @param:  cam_coord是相机系的坐标, rotate_world_cam是从世界系转相机系的旋转矩阵
 *
 * @return: 装甲板在云台系的位置和姿态
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d>
PredictorPose::cam2ptz(std::pair<Eigen::Vector3d, Eigen::Vector3d> &cam_coord, GimbalPose gm)
{
    cam_coord.first[0] += X_BIAS;
    cam_coord.first[1] += Y_BIAS;
    cam_coord.first[2] += Z_BIAS;
    Eigen::Matrix3d pitch_rotation_matrix_t;
    pitch_rotation_matrix_t
        << 1,
        0, 0,
        0, std::cos((gm.pitch * CV_PI) / 180), std::sin((gm.pitch * CV_PI) / 180),
        0, -std::sin((gm.pitch * CV_PI / 180)), std::cos((gm.pitch * CV_PI) / 180);

    Eigen::Matrix3d yaw_rotation_matrix_t;
    yaw_rotation_matrix_t
        << std::cos((gm.yaw * CV_PI) / 180),
        0, std::sin((gm.yaw * CV_PI) / 180),
        0, 1, 0,
        -std::sin((gm.yaw * CV_PI) / 180), 0, std::cos((gm.yaw * CV_PI) / 180);

    /**
     * 写两种矩阵的原因是因为位置和姿态所用坐标系不同
     * 用欧拉较或泰特布莱恩角表示旋转，顺序十分重要，一般是X-Y-Z，但RM这种小角度，绕定轴动轴都一样顺序什么样结果都一样
     * 可以动手试试
     */
    Eigen::Vector3d ptz_coord = yaw_rotation_matrix_t * pitch_rotation_matrix_t * cam_coord.first;
    Eigen::Matrix3d transform_vector;
    transform_vector = yaw_rotation_matrix_t * pitch_rotation_matrix_t;
    transform_vector_ = transform_vector.inverse();

    Eigen::Vector3d rotation;
    rotation << cam_coord.second[0], cam_coord.second[1], cam_coord.second[2];

    std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
    pose.first = ptz_coord;
    pose.second = rotation;

    return pose;
}
/**
 * @brief:选择最佳装甲板
 */
bool PredictorPose::Need_better_target(const std::vector<TRTInferV1::DetectObject> &objects, TRTInferV1::DetectObject last_obj_)
{
    TRTInferV1::DetectObject Current_obj = ArmorSelect(objects);
    if (Current_obj.label == 1 || Current_obj.label == 10)
    { // 选择英雄
        std::cout << "ChoosedHero" << std::endl;
        return true;
    }
    else if (Current_obj.label == 0 || Current_obj.label == 9)
    { // 选择哨兵
        std::cout << "ChoosedSentry" << std::endl;
        return true;
    }
    else if (last_obj_.label != Current_obj.label && Current_obj.coord[2] > last_obj_.coord[2])
    { // 选择与当前装甲板不同类且距离更近的装甲板
        std::cout << "ChoosedDiffNear" << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief : 预测初始化状态
 * @author AlexLi
 * @date : 2024-03-03
 * @return GimbalPose
 */
GimbalPose PredictorPose::init(std::vector<DetectObject> &objects,GimbalPose imu_data)
{
    // 选择一个装甲板
    TRTInferV1::DetectObject ArmorInit = ArmorSelect(objects);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
    world_cam_pose = pnp_solve_->poseCalculation(ArmorInit);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
    cam_ptz_pose = cam2ptz(world_cam_pose, imu_data);
    bullteFlyTime(cam_ptz_pose.first);
    last_pose_ = cam_ptz_pose; // 记录这一时刻，为下一时刻预测做准备
    last_location_ = cam_ptz_pose.first;
    track_cnt++; // 初始化跟踪计数器
    last_time_ = current_time_;
    GimbalPose gm = gm_ptz;
    velocities_.clear();
    last_eular_ = gm;
    return gm;
}

/**
 * @brief : 预测丢失状态
 * @author AlexLi
 * @date : 2024-03-03
 * @return GimbalPose
 */
GimbalPose PredictorPose::Loss()
{
    velocities_.clear();
    track_cnt = 0;
    ekf.P = Eigen::Matrix<double, 9, 9>::Identity();
    return last_eular_;
}

/**
 * @brief : 预测短暂丢失状态
 * @param  imu_data
 * @param  objects
 * @param  time
 * @return GimbalPose
 */
GimbalPose PredictorPose::TEMP_LOSS(double current_time)
{
    Eigen::Vector3d current_pose;
    current_pose[0] = last_location_[0] + last_velocity_[0] * (current_time - last_time_);
    current_pose[1] = last_location_[1] + last_velocity_[1] * (current_time - last_time_);
    current_pose[2] = last_location_[2] + last_velocity_[2] * (current_time - last_time_);
    double fly_t = bullteFlyTime(current_pose);

    Eigen::Vector3d predict_pose;
    predict_pose[0] = current_pose[0] + last_velocity_[0] * fly_t;
    predict_pose[1] = current_pose[1] + last_velocity_[1] * fly_t;
    predict_pose[2] = current_pose[2] + last_velocity_[2] * fly_t;

    bullteFlyTime(predict_pose);
    last_time_ = current_time_;
    predict_location_ = predict_pose;
    GimbalPose gm = last_eular_;
    return gm;
}

/**
 * @brief : 预测跟踪状态
 * @param  imu_data
 * @param  objects
 * @param  time
 * @return GimbalPose
 */
GimbalPose
PredictorPose::Track(const std::vector<TRTInferV1::DetectObject> &objects, double current_time,GimbalPose imu_data)
{
    TRTInferV1::DetectObject ArmorTrack = ArmorSelect(objects);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
    world_cam_pose = pnp_solve_->poseCalculation(ArmorTrack);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
    cam_ptz_pose = cam2ptz(world_cam_pose, imu_data);
    #ifdef Better_Target
    //检查是否需要重新初始化
    if (Need_better_target(objects, ArmorTrack))
    {
        std::cout << "装甲板NeedReInit!!!" << std::endl;
        state = ARMOR_STATE_::LOSS;
        return last_eular_;
    }
    #endif
    if (std::sqrt(std::pow(cam_ptz_pose.first[0] - last_location_[0], 2) +
                  std::pow(cam_ptz_pose.first[1] - last_location_[1], 2) +
                  std::pow(cam_ptz_pose.first[2] - last_location_[2], 2)) >= 0.7)
    {
        state = ARMOR_STATE_::LOSS;
        return last_eular_;
    }
    //========================EKF========================//
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    Predict predictfunc{};
    Measure measure;
    if (isnan(ekf.P.sum()))
    {
        ekf.P = Eigen::Matrix<double, 9, 9>::Identity();
    }
    #ifdef Best_Yaw // 重投影纠正pnp的yaw
    yaw = best_yaw(ArmorTrack); 
    #endif
    predictfunc.delta_t = (current_time_ - last_time_);
    if (predictfunc.delta_t != 0)
    {
        yaw_v = (yaw - last_yaw) / predictfunc.delta_t;
    }
    else
    {
        yaw_v = 0;
    }
    Eigen::Matrix<double, 9, 1> Xh;
    Xh
        << last_location_[0],
        last_velocity_[0], last_location_[1], last_velocity_[1], last_location_[2], last_velocity_[2], yaw, yaw_v, r;
    ekf.init(Xh);
    Eigen::Matrix<double, 9, 1> Xr;
    Xr << cam_ptz_pose.first(0, 0), 0, cam_ptz_pose.first(1, 0), 0, cam_ptz_pose.first(2, 0), 0, yaw, 0, r;
    Eigen::Matrix<double, 4, 1> Yr;
    measure(Xr.data(), Yr.data());                            // 转化成pitch,yaw,distance
    ekf.predict(predictfunc);                                 // 更新预测器，此时预测器里的是预测值
    Eigen::Matrix<double, 9, 1> Xe = ekf.update(measure, Yr); // 更新滤波器，输入真实的球面坐标 Yr
    if (!isnan(Xe[0]) && !isnan(Xe[2]) && !isnan(Xe[4]) && !isnan(Xe[6]) && !isnan(Xe[8]))
    {
        //        cam_ptz_pose.first[0] = Xe[0];
        //        cam_ptz_pose.first[1] = Xe[2];
        //        cam_ptz_pose.first[2] = Xe[4];
    }
    else
    {
        ekf.P = Eigen::Matrix<double, 9, 9>::Identity();
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0);
    std::cout << "[EKF time is : " << time_run.count() * 1000 << " ]" << std::endl;
    //========================EKF========================//
    double fly_t = bullteFlyTime(cam_ptz_pose.first);

    Eigen::Vector4d current_state; // 现在的速度: 直接计算的速度，未经过拟合
    current_state[0] = cam_ptz_pose.first[0];
    current_state[1] = cam_ptz_pose.first[1];
    current_state[2] = cam_ptz_pose.first[2];
    current_state[3] = current_time_;

    Eigen::Vector3d now_v;
    if (velocities_.size() < velocities_deque_size_)
    {
        velocities_.push_back(current_state);
    }
    else
    {
        velocities_.pop_front();
        velocities_.push_back(current_state);
    }
    now_v = CeresVelocity(velocities_);

    if (isnan(now_v[0]) || isnan(now_v[1]) || isnan(now_v[2]))
    {
        now_v = last_velocity_;
    }

    // now_v[0] = 0;
    now_v[1] = 0;
    now_v[2] = 0;
    // cv::Point3f point;
    // point.x = now_v[0];
    //drawCurveData(point);

    Eigen::Vector3d predict_location;
    toVofa.vx_camera = now_v[0];
    std::cout << "[ vx: " << now_v[0] << " vy: " << now_v[1] << " vz: " << now_v[2] << " ]" << std::endl;

    predict_location[0] = cam_ptz_pose.first[0] + (now_v[0] * (fly_t));
    predict_location[1] = cam_ptz_pose.first[1] + (now_v[1] * (fly_t));
    predict_location[2] = cam_ptz_pose.first[2] + (now_v[2] * (fly_t));

    bullteFlyTime(predict_location);
    last_location_ = cam_ptz_pose.first;
    last_time_ = current_time_;
    last_velocity_ = now_v;
    last_state_ = state;
    last_pose_ = cam_ptz_pose;
    last_obj = ArmorTrack;

    predict_location_ = predict_location;

    GimbalPose gm = gm_ptz;
    gm.yaw = gm.yaw + pit_corangle;
    gm.pitch = gm.pitch + yaw_corangle;
    last_eular_ = gm;
    return gm;
}

/**
 * @brief : 预测陀螺状态
 */
GimbalPose PredictorPose::AntiGryo()
{
    #ifdef NT_Gyro
       if (is_gyro_) {
           float distance_;
           distance_ = std::sqrt(predict_location[0] * predict_location[0] + predict_location[2] * predict_location[2]);
           float yaw_pre = std::asin(predict_location[0] / distance_) * 180 / CV_PI;
           float distance_right;
           distance_right = std::sqrt(right_switch[0] * right_switch[0] + right_switch[2] * right_switch[2]);
           float yaw_right = std::asin(right_switch[0] / distance_right) * 180 / CV_PI;
    
           float distance_left;
           distance_left = std::sqrt(left_switch[0] * left_switch[0] + left_switch[2] * left_switch[2]);
           float yaw_left = std::asin(left_switch[0] / distance_left) * 180 / CV_PI;
    
           if (yaw_pre > yaw_right) {
               predict_location = left_switch;
           } else if (yaw_pre < yaw_left) {
               predict_location = right_switch;
           }
       }
    #endif
    return last_eular_;
}
/**
 * @brief:  预测主程序
 *
 * @author: AlexLi
 *
 * @param:  imu_data是IMU姿态解算的结果, objects是本帧所有的装甲板，time是图像的时间戳
 *
 * @return: 经过预测后云台上台和偏航的角度
 */
GimbalPose PredictorPose::run(GimbalPose &imu_data, std::vector<DetectObject> &objects, double time)
{
    current_time_ = time;
    std::cout << static_cast<int>(state) << std::endl;

    if (objects.empty())
    {
        if (state == ARMOR_STATE_::TRACK || state == ARMOR_STATE_::GRYO || state == ARMOR_STATE_::TEMP_LOSS)
        {
            state = ARMOR_STATE_::TEMP_LOSS;
            brief_loss_cnt++;
            if (brief_loss_cnt > NT_LOSS_MAXQ)
            {
                state = ARMOR_STATE_::LOSS;
                return Loss();
            }
            else
            {
                return TEMP_LOSS(current_time_);
            }
        }
        else if (state == ARMOR_STATE_::LOSS)
        {
            state = ARMOR_STATE_::LOSS;
            return Loss();
        }
    }
    else
    {
        brief_loss_cnt = 0;
        if (state == ARMOR_STATE_::LOSS || track_cnt <= NT_TRACK_MAXQ)
        {
            std::cout << "装甲板INTI!!!" << std::endl;
            state = ARMOR_STATE_::INIT;
            is_gyro_ = false;
            return init(objects,imu_data);
        }

        if ((state == ARMOR_STATE_::INIT && track_cnt > NT_TRACK_MAXQ) || state == ARMOR_STATE_::TEMP_LOSS)
        {
            std::cout << "装甲板StratTrack!!!" << std::endl;
            state = ARMOR_STATE_::TRACK;
        }

        if (state == ARMOR_STATE_::TRACK)
        {
            std::cout << "装甲板TRACKING!!!" << std::endl;
            if (is_gyro_)
            {
                state = ARMOR_STATE_::GRYO;
            }
            return Track(objects, current_time_,imu_data);
        }
        if (state == ARMOR_STATE_::GRYO)
        {
            std::cout << "GRYO!!!" << std::endl;
            return AntiGryo();
        }
    }
    // 这里不应该执行到
    return last_eular_;
}
#endif


#ifndef NT
/**
 * @brief:  预测主程序
 *
 * @author: liqianqi
 *
 * @param:  imu_data是IMU姿态解算的结果, objects是本帧所有的装甲板，time是图像的时间戳
 *
 * @return: 经过预测后云台上台和偏航的角度
 */
GimbalPose PredictorPose::run_predict(GimbalPose &imu_data, std::vector<DetectObject> &objects, double time)
{
	bool terminal_flag = true; //终端是否显示输出内容

	// 更新IMU数据和当前时间
	imu_data_ = imu_data;
	current_time_ = time;

//---------------------------------cam2ptz start--------------------------------------
	// 计算俯仰角（pitch）的旋转矩阵
	Eigen::Matrix3d pitch_rotation_matrix_t;
	pitch_rotation_matrix_t
		<< 1,
		0, 0,
		0, std::cos((imu_data_.pitch * CV_PI) / 180), std::sin((imu_data_.pitch * CV_PI) / 180),
		0, -std::sin((imu_data_.pitch * CV_PI / 180)), std::cos((imu_data_.pitch * CV_PI) / 180);

	// 计算偏航角（yaw）的旋转矩阵
	Eigen::Matrix3d yaw_rotation_matrix_t;
	yaw_rotation_matrix_t
		<< std::cos((imu_data_.yaw * CV_PI) / 180),
		0, std::sin((imu_data_.yaw * CV_PI) / 180),
		0, 1, 0,
		-std::sin((imu_data_.yaw * CV_PI) / 180), 0, std::cos((imu_data_.yaw * CV_PI) / 180);

	// 计算总的旋转矩阵，并求其逆矩阵
	Eigen::Matrix3d transform_vector;
	transform_vector =  yaw_rotation_matrix_t * pitch_rotation_matrix_t;
	transform_vector_ = transform_vector.inverse();
//-------------------------------cam2ptz end--------------------------------------------

	// 如果这一帧没有装甲板，先看看init_是true还是false
	// 是true

/**
 * @brief 处理目标丢失状态下的逻辑
 * 
 * 该函数在目标丢失时执行一系列操作，包括更新丢失计数器、状态切换、预测目标位置等。
 * 如果目标丢失时间过长，函数会返回最后一帧的云台角度，避免剧烈晃动。
 * 
 * @return GimbalPose 返回云台的预测角度或最后一帧的角度。
 */
if (!objects.size())
{
    // 增加丢失计数器，并限制其最大值为250
    loss_cnt_++;
    if (loss_cnt_ > loss_cnt_max)
    {
        loss_cnt_ = loss_cnt_max;
    }

    // 如果当前状态已经是LOSS状态，则直接返回最后一帧的云台角度
    if (state_ == ARMOR_STATE_::LOSS)
    {
        find_switch = false;
		// 输出丢失装甲板的警告信息
		std::cout << REDCOLOR << "LOSS ARMOR!!!" << std::endl;
		std::cout << WHITECOLOR << std::endl;
        return last_eular_;
    }
//-------------------------------loss start--------------------------------------------
    // 如果丢失计数器超过pre_los_maxq，则切换到LOSS状态，并返回最后一帧的云台角度
	if (loss_cnt_ > pre_los_maxq)
	{
		// 更新系统状态为丢失状态，并记录当前状态
		state_ = ARMOR_STATE_::LOSS;
		last_state_ = state_;
		
		// 初始化标志位，避免系统在丢失状态下进行不必要的操作
		init_ = true;
		find_switch = false;
		
		// 清除速度、偏航角和时间缓冲区，以重置系统状态
		velocities_.clear();
		yaw_rad_.clear();
		time_buff_.clear();
		
		// 输出丢失装甲板的警告信息
		std::cout << REDCOLOR << "LOSS ARMOR!!!" << std::endl;
		std::cout << WHITECOLOR << std::endl;

		// 返回最后一帧的云台角度，避免剧烈晃动
		return last_eular_;
	}
//-------------------------------loss end--------------------------------------------
//-------------------------------temploss start--------------------------------------------
    // 如果丢失计数器未超过pre_los_maxq，则保持TRACK状态，并进行目标位置预测
    state_ = ARMOR_STATE_::TRACK;
    std::cout << BOLDBLACK << "TEMP LOSS TRACKING!!!" << std::endl;
    std::cout << WHITECOLOR << std::endl;
    last_state_ = state_;

    // 根据上一帧位置和速度在做预测，100次  !!!     
    // 计算当前时刻的位置
    Eigen::Vector3d current_pose;
    current_pose[0] = last_location_[0] + last_velocity_[0] * (current_time_ - last_time_);
    current_pose[1] = last_location_[1] + last_velocity_[1] * (current_time_ - last_time_);
    current_pose[2] = last_location_[2] + last_velocity_[2] * (current_time_ - last_time_);
    
    // 计算子弹飞行时间
    float fly_t = bullteFlyTime(current_pose);

    // 根据预测的飞行时间进一步预测目标位置
    Eigen::Vector3d predict_pose;
    predict_pose[0] = current_pose[0] + last_velocity_[0] * fly_t;
    predict_pose[1] = current_pose[1] + last_velocity_[1] * fly_t;
    predict_pose[2] = current_pose[2] + last_velocity_[2] * fly_t;

    // 根据不同的条件更新预测位置和飞行时间
    bullteFlyTime(current_pose);
    //bullteFlyTime(predict_pose); lsn

	#ifdef Test2
	last_time_ = current_time_; //lsn
	#endif
    last_location_ = current_pose;
    predict_location_ = predict_pose;

    // 更新云台角度并返回
    GimbalPose gm = gm_ptz;
    last_eular_ = gm;

    return gm;
}
//-------------------------------temploss end--------------------------------------------

	loss_cnt_ = 0;

//*****************************init start*************************************** */

	if (init_)
	{
	    // 打印初始化完成信息
	    std::cout << "init finished!!!" << std::endl;
	    
	    // 调用初始化函数
	    init();
	    
	    // 设置系统状态为跟踪状态，并记录当前状态
	    state_ = ARMOR_STATE_::TRACK;
	    last_state_ = state_;
	    last_obj_nums = objects.size();

//-------------------------------ArmorSelect1 start----------------------------------------- 
	    // 计算每个装甲板到图像中心的距离，并选择距离最近的装甲板
	    float distances[objects.size()];
	    for (int i = 0; i < objects.size(); i++)
	    {
	        cv::Point2f center = (objects[i].pts[0] + objects[i].pts[2]) / 2;
	        distances[i] = (center.x - 512) + (center.y - 640);
	    }
	
	    int index = 0;
	    for (int i = 1; i < objects.size(); i++)
	    {
	        if (distances[i] < distances[index])
	            index = i;
	    }
	    DetectObject obj = objects[index];
//-------------------------------ArmorSelect1 end----------------------------------------- 
	
	    // 计算目标装甲板在世界坐标系和相机坐标系中的位姿
	    std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
	    world_cam_pose = pnp_solve_->poseCalculation(obj,terminal_flag);
	
	    std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose; 
		//一个std::pair对象包含两个 Eigen::Vector3d 类型的向量，分别表示相机的姿态信息：
		//first: 表示相机的平移向量（Translation Vector），通常包含相机在世界坐标系中的位置信息（x, y, z）
		//second: 表示相机的旋转向量（Rotation Vector），通常包含相机的旋转角度或方向信息（roll, pitch, yaw）
	
	    #ifdef Test3
	    cam_ptz_pose = cam2ptz(world_cam_pose.first, pnp_solve_->rotate_world_cam_);//原程序未启用
	    #endif
	    
	    // 根据IMU数据计算相机到云台的位姿
	    cam_ptz_pose.first = cam3ptz(imu_data_, world_cam_pose.first,terminal_flag);
	    cam_ptz_pose.second = world_cam_pose.second;
	
	    // 打印目标在世界坐标系中的位置信息
	    std::cout << "[world x]: " << cam_ptz_pose.first[0] << "  "
	              << "[world y]: " << cam_ptz_pose.first[1] << "  [world z]: " << cam_ptz_pose.first[2] << std::endl;
	
	    // 记录当前位姿、时间、目标信息等，并返回云台位姿
	    last_pose_ = cam_ptz_pose; // 记录这一时刻，为下一时刻预测做准备
	    init_ = false;
	    last_location_ = cam_ptz_pose.first;
	    
	    // 计算子弹飞行时间
	    float fly_t = bullteFlyTime(cam_ptz_pose.first);
	    last_time_ = current_time_;
	    GimbalPose gm = gm_ptz;
	    velocities_.clear();
	    last_id = obj.label;
	    last_obj = obj;
	    last_eular_ = gm;
	    return gm;
	}
//*****************************init end*************************************** */

//*****************************Track start*************************************** */
	std::cout << "continue track armor!!!" << std::endl;

	// 选择一个装甲板
	// 当卡方检验过大时仍然要打开初始化开关
	// 将状态设置为跟踪状态
	state_ = ARMOR_STATE_::TRACK;

	// 从检测到的对象中选择一个装甲板
	DetectObject obj = ArmorSelect(objects,terminal_flag);

	// 计算装甲板的像素中心点
	obj_pixe_ = (obj.pts[0] + obj.pts[2]) / 2;

	// 定义相机与云台姿态的变量
	std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;

	// 计算当前装甲板与相机之间的距离
	float distance_;
	distance_ = std::sqrt(obj.coord[0] * obj.coord[0] + obj.coord[2] * obj.coord[2]);

	// 计算当前装甲板的偏航角
	float yaw_now = std::asin(obj.coord[0] / distance_) * 180 / CV_PI;

	// 计算上一次装甲板与相机之间的距离
	float distance_last;
	distance_last = std::sqrt(last_obj.coord[0] * last_obj.coord[0] + last_obj.coord[2] * last_obj.coord[2]);

	// 计算上一次装甲板的偏航角
	float yaw_last = std::asin(last_obj.coord[0] / distance_last) * 180 / CV_PI;


//========================antigyro1 start========================//
#ifdef Gyro1
	// 初始化标志变量为false
	switch_flag = false;
	
	// 检查当前对象与上一个对象的位置是否发生切换
	if (isSwitch(obj.coord, last_obj.coord))
	{
	    // 如果发生切换，增加切换计数器并设置标志为true
	    cnt_switch++;
	    switch_flag = true;
	    Eigen::Vector3d now_record = obj.coord;
	    velocities_.clear();
	
	    // 如果切换次数小于6次且未使用陀螺仪模式，根据上一次的速度更新当前对象的位置和姿态
	    if (cnt_switch < 6 && is_gyro_ == false)
	    {
	        obj.coord = last_obj.coord + last_velocity_ * (current_time_ - last_time_);
	        obj.pyr = last_obj.pyr;
	    }
	
	    // 如果当前时间与上次切换时间的间隔大于1秒，重置陀螺仪计数器并更新上次切换时间
	    if(std::abs(current_time_ - last_switch_time_) > 1.0)
	    {
	        is_gyro_cnt_ = 0;
	        last_switch_time_ = current_time_;
	    }

	
	    // 如果当前时间与上次切换时间的间隔在0.15秒到1秒之间，增加陀螺仪计数器
	    if((std::abs(current_time_ - last_switch_time_)) > 0.15
	        && (current_time_ - last_switch_time_) < 1.0)
	    {
	        is_gyro_cnt_++;
	    }
	
	    // 如果陀螺仪计数器大于4，启用陀螺仪模式，并根据当前和上一个 yaw 角度确定右侧和左侧切换点
	    if(is_gyro_cnt_ > 4)
	    {
	        is_gyro_ = true;
            #ifdef NEXT_Gyro
            // +++ 新增陀螺模式初始化逻辑
            if (!is_gyro_locked_) 
            {  // 首次进入时清空缓存
                mid_pt_buffer_.clear();
                ave_mid_pt_.setZero();

                if (yaw_now > yaw_last)
                {
                    right_switch_ = obj.coord;
                    left_switch_ = last_obj.coord;
                }
                else
                {
                    right_switch_ = last_obj.coord;
                    left_switch_ = obj.coord;
                }
            }
            #endif
            #ifndef NEXT_Gyro
	        if (yaw_now > yaw_last)
	        {
	            right_switch_ = obj.coord;
	            left_switch_ = last_obj.coord;
	        }
	        else
	        {
	            right_switch_ = last_obj.coord;
	            left_switch_ = obj.coord;
	        }
            #endif
	    }
	    else
	    {
	        // 否则，禁用陀螺模式
	        is_gyro_ = false;
	    }
	
	    // 更新上次切换时间为当前时间
	    last_switch_time_ = current_time_;
	}

	// 输出 time_buff_ 的大小
	std::cout << "time_buff_ size is: " << time_buff_.size() << std::endl;
	
	// 如果 switch_flag 为 false，重置 cnt_switch
	if (!switch_flag)
	{
	    cnt_switch = 0;
	}
	
	// 如果当前时间与上次切换时间的差值大于 1.0 秒，重置陀螺仪状态
	if(std::abs(current_time_ - last_switch_time_) > 1.0)
	{
	    is_gyro_ = false;
	    is_gyro_cnt_ = 0;
	}
#endif

	// 计算当前时间与上次切换时间的时间差
	time_diff = current_time_ - last_switch_time_;
	//========================antigyro1 end========================//
	
	// 更新相机云台姿态信息
	// cam_ptz_pose.first 存储目标对象的坐标信息
	cam_ptz_pose.first = obj.coord;
	// cam_ptz_pose.second 存储目标对象的姿态信息（俯仰、偏航、滚转）
	cam_ptz_pose.second = obj.pyr;
	

	// cam_ptz_pose = world_cam_pose;//lqq禁用
	std::cout << "[world x]: " << cam_ptz_pose.first[0] << "  "
			  << "[world y]: " << cam_ptz_pose.first[1] << "  [world z]: " << cam_ptz_pose.first[2] << std::endl;


//-----------------------Normal Kalman start---------------------
	// 设置卡尔曼滤波器的方向变化标志
	kf.is_change_direction_ = switch_flag;
	
	// 使用卡尔曼滤波更新目标位置
	cam_ptz_pose.first = kf.KalmanMainRun(cam_ptz_pose.first, last_velocity_);
//-----------------------Normal Kalman end---------------------

	std::cout << "world coord solve finished" << std::endl;
	
	// 计算子弹飞行时间
	float fly_t = bullteFlyTime(cam_ptz_pose.first);
	
	std::cout << "the first fly time geted" << std::endl;
	
	// 创建当前状态向量，包含目标位置和当前时间
	Eigen::Vector4d current_state; // 现在的速度: 直接计算的速度，未经过拟合
	current_state[0] = cam_ptz_pose.first[0];
	current_state[1] = cam_ptz_pose.first[1];
	current_state[2] = cam_ptz_pose.first[2];
	current_state[3] = current_time_;
	
	// 更新速度队列，保持队列大小不超过最大容量
	if (velocities_.size() < velocities_deque_size_)
	{
	    velocities_.push_back(current_state);
	}
	else
	{
	    velocities_.pop_front();
	    velocities_.push_back(current_state);
	}
	
	// 使用Ceres库拟合当前速度
	Eigen::Vector3d now_v = CeresVelocity(velocities_);
	
	// 如果拟合结果无效，则使用上一次的速度
	if (isnan(now_v[0]) || isnan(now_v[1]) || isnan(now_v[2]))
	{
	    now_v = last_velocity_;
	}
	
	// 限制速度的某些分量（例如，y轴速度为0）
	// now_v = (last_velocity_ + now_v) / 2;
	// now_v[0] = 0; // 一般就是注释上的，解除可能导致红点斜向来会折返
	now_v[1] = 0;
	#ifdef Test1
	now_v[2] = 0; //注释上可能出现小幅度斜着预测的怪异问题,但不注释会出现p轴无法预测且yaw轴过预测的问题
	#endif
	
	// 将速度向量转换为cv::Point3f格式
	cv::Point3f point;
	point.x = now_v[0];
	

	// 预测目标未来位置
	Eigen::Vector3d predict_location;
#ifndef NEXT_Gyro
	// std::cout << "[ vx: " << now_v[0] << " vy: " << now_v[1] << " vz: " << now_v[2] << " ]" << std::endl;
	
	predict_location[0] = cam_ptz_pose.first[0] + (now_v[0] * (fly_t));
	predict_location[1] = cam_ptz_pose.first[1] + (now_v[1] * (fly_t));
	predict_location[2] = cam_ptz_pose.first[2] + (now_v[2] * (fly_t));
#endif

#ifdef NEXT_Gyro
    // +++ 修改为
    if (is_gyro_ && is_gyro_locked_) {
        // 使用平均值作为预测目标
        predict_location = ave_mid_pt_;
    } else {
	predict_location[0] = cam_ptz_pose.first[0] + (now_v[0] * (fly_t));
	predict_location[1] = cam_ptz_pose.first[1] + (now_v[1] * (fly_t));
	predict_location[2] = cam_ptz_pose.first[2] + (now_v[2] * (fly_t));
    }
    // ---
#endif

// ========================antigyro2 start========================//
#ifdef Gyro2 //小陀螺自描（存在问题，目前禁用）
	/**
	 * @brief 该函数用于处理陀螺仪模式下的预测位置计算和切换逻辑。
	 * 
	 * 该函数根据陀螺仪模式（is_gyro_）的状态，计算预测位置的偏航角（yaw），并与左右切换位置的偏航角进行比较，
	 * 决定是否切换到左或右位置。此外，如果启用了自动开火模式（Auto_Fire_Infantry），还会计算中间位置并判断是否开火。
	 */
	if (is_gyro_)
	{
	    // 计算预测位置的偏航角
	    float distance_ = std::sqrt(predict_location[0] * predict_location[0] + predict_location[2] * predict_location[2]);
	    float yaw_pre = std::asin(predict_location[0] / distance_) * 180 / CV_PI;
	
	    // 计算右切换位置的偏航角
	    float distance_right = std::sqrt(right_switch_[0] * right_switch_[0] + right_switch_[2] * right_switch_[2]);
	    float yaw_right = std::asin(right_switch_[0] / distance_right) * 180 / CV_PI;
	
	    // 计算左切换位置的偏航角
	    float distance_left = std::sqrt(left_switch_[0] * left_switch_[0] + left_switch_[2] * left_switch_[2]);
	    float yaw_left = std::asin(left_switch_[0] / distance_left) * 180 / CV_PI;
	
	    // 判断左右切换位置的距离是否小于阈值，决定是否切换到左或右位置
	    if(std::sqrt(std::pow(left_switch_[0] - right_switch_[0],2) 
	        + std::pow(left_switch_[1] - right_switch_[1],2) 
	        + std::pow(left_switch_[2] - right_switch_[2],2)) < 0.5)
	    {
	        if (yaw_pre > yaw_right)
	        {
	            predict_location = left_switch_; // 切换到左位置
	        }
	        else if (yaw_pre < yaw_left)
	        {
	            predict_location = right_switch_; // 切换到右位置
	        }
	    }
	    else
	    {
	        is_gyro_ = false; // 如果距离大于阈值，则退出陀螺仪模式
	    }
	
	    // 如果启用了自动开火模式，计算中间位置并判断是否开火
	    #ifdef Auto_Fire_Gyro
	    if(isSwitch(obj.coord, last_obj.coord))
	    {
	        // 计算中间位置
	        mid_pt_[0] = (left_switch_[0] + right_switch_[0]) / 2;
	        mid_pt_[1] = (left_switch_[1] + right_switch_[1]) / 2;
	        mid_pt_[2] = (left_switch_[2] + right_switch_[2]) / 2;
            #ifndef NEXT_Gyro
	        // 计算子弹飞行时间
	        bullteFlyTime(mid_pt_);
	
	        // 计算中间位置的偏航角
	        float distance_mid_ = std::sqrt(mid_pt_[0] * mid_pt_[0] + mid_pt_[2] * mid_pt_[2]);
	        float yaw_mid_ = std::asin(mid_pt_[0] / distance_mid_) * 180 / CV_PI;
	
	        // 计算预测位置的偏航角
	        float distance_pre_ = std::sqrt(predict_location[0] * predict_location[0] + predict_location[2] * predict_location[2]);
	        float yaw_pre_ = std::asin(predict_location[0] / distance_pre_) * 180 / CV_PI;
	
	        // 判断中间位置与预测位置的偏航角差是否小于阈值，决定是否开火
	        if(std::abs(yaw_mid_ - yaw_pre_) < 2)
	        {
	            is_fire = true; // 开火
	        }
	        else
	        {
	            is_fire = false; // 不开火
	        }
            #endif
            #ifdef NEXT_Gyro
            // +++ 新增中间点缓存处理
            if (is_gyro_ && mid_pt_buffer_.size() < GYRO_SAMPLE_NUM) {
                mid_pt_buffer_.push_back(mid_pt_);
            }
            
            // 达到采样次数时计算平均值
            if (mid_pt_buffer_.size() == GYRO_SAMPLE_NUM && !is_gyro_locked_) {
                ave_mid_pt_.setZero();
                for (const auto& pt : mid_pt_buffer_) {
                    ave_mid_pt_ += pt;
                }
                ave_mid_pt_ /= GYRO_SAMPLE_NUM;
                is_gyro_locked_ = true;  // 进入锁定状态
            }
            
            // 计算当前中间点与平均值的yaw差
            float yaw_diff = calculateYawDiff(mid_pt_, ave_mid_pt_);
            float yaw_switch_diff = std::abs(yaw_right - yaw_left);
            
            // 如果偏差超过切换差值则重置锁定
            if (yaw_diff > yaw_switch_diff * 0.8) {
                is_gyro_locked_ = false;
                mid_pt_buffer_.clear();
            }
            // ---

            // 原有开火判断逻辑修改为
            if(is_gyro_locked_){
                // 使用平均值计算开火条件
                float distance_to_center = calculateDistance(predict_location, ave_mid_pt_);
                is_fire = (distance_to_center < GYRO_FIRE_THRESH);
            }
            #endif
	    }
	    #endif
	};
	#endif
// ========================antigyro2 end========================//

	//std::cout << "predict expression " << (now_v[0] * (fly_t)) * 100 << " cm " << std::endl;
	move_ = (now_v[0] * (fly_t)) * 100;

#ifndef NEXT_Gyro
	bullteFlyTime(predict_location); //此处为predict_location时处理结果为预测位置
#endif
#ifdef NEXT_Gyro
    // +++ 修改为
    if (!is_gyro_ || !is_gyro_locked_) { // 非陀螺模式使用常规预测
        bullteFlyTime(predict_location);
    } else { // 陀螺模式使用平均值解算
        bullteFlyTime(ave_mid_pt_); 
    }
    
    // ---
    // // 修改步骤6弹道解算调用
    // if (is_gyro_ && is_gyro_locked_) { 
    //     // 使用锁定中心点解算
    //     bullteFlyTime(ave_mid_pt_);  
    // } else { 
    //     // 常规预测解算
    //     bullteFlyTime(predict_location);
    // }

std::cout << "[弹道解算] 目标位置: " 
          << (is_gyro_locked_ ? ave_mid_pt_ : predict_location).transpose()
          << std::endl;

#endif


#ifdef Outpost
	/**
	 * @brief 击打前哨站（zyl）（lqq未写）
	 * 
	 * @param obj 当前对象，包含类别信息和坐标信息。
	 * @param last_obj 上一个对象，包含坐标信息。
	 * @param find_switch 标志位，用于判断是否已经找到角度变化的中间点。
	 * @param mid_pt 存储中间点的坐标。
	 * @param predict_location 预测的位置坐标，用于与中间点进行比较。
	 * @param is_fire 标志位，用于判断是否需要触发开火操作。
	 */
	//if(obj.label == 6 || obj.label == 15) //6还是10待测
	if(obj.label == 10 || obj.label == 15)
	{
	    // 计算当前对象的距离和偏航角
	    float distance_ = std::sqrt(obj.coord[0] * obj.coord[0] + obj.coord[2] * obj.coord[2]);
	    float yaw_now = std::asin(obj.coord[0] / distance_) * 180 / CV_PI;
	
	    // 计算上一个对象的距离和偏航角
	    float distance_last = std::sqrt(last_obj.coord[0] * last_obj.coord[0] + last_obj.coord[2] * last_obj.coord[2]);
	    float yaw_last = std::asin(last_obj.coord[0] / distance_last) * 180 / CV_PI;
	
	    // 如果尚未找到角度变化的中间点
	    if(!find_switch)
	    {
	        // 如果当前对象与上一个对象的偏航角变化超过3度，则标记找到中间点，并计算中间点坐标
	        if (std::abs(yaw_now - yaw_last) > 3.0)
	        {
	            find_switch = true;
	            mid_pt[0] = (last_obj.coord[0] + obj.coord[0])/2;
	            mid_pt[1] = obj.coord[1];
	            mid_pt[2] = obj.coord[2];
	        }
	    }
	    else
	    {
	        // 计算中间点的距离和偏航角
	        bullteFlyTime(mid_pt);
	        float distance_mid = std::sqrt(mid_pt[0] * mid_pt[0] + mid_pt[2] * mid_pt[2]);
	        float yaw_mid = std::asin(mid_pt[0] / distance_mid) * 180 / CV_PI;
	
	        // 计算预测位置的距离和偏航角
	        float distance_pre = std::sqrt(predict_location[0] * predict_location[0] + predict_location[2] * predict_location[2]);
	        float yaw_pre = std::asin(predict_location[0] / distance_pre) * 180 / CV_PI;
	
	        // 如果中间点与预测位置的偏航角差异小于1度，则触发开火操作
	        if(std::abs(yaw_mid - yaw_pre) < 1)
	        {
	            is_fire = true;
	        }
	        else
	        {
	            is_fire = false;
	        }
	    }
	};
#endif

	// 更新最后的位置、时间、速度、状态和姿态信息
	last_location_ = cam_ptz_pose.first;
	last_time_ = current_time_;
	last_velocity_ = now_v;
	last_state_ = state_;
	last_pose_ = cam_ptz_pose;
	predict_location_ = predict_location;

	// 根据弹道偏向调整云台的偏航和俯仰角度
	GimbalPose gm = gm_ptz;
	gm.yaw = gm.yaw + yaw_corangle;
	gm.pitch = gm.pitch + pit_corangle;
	last_obj = obj;
	last_eular_ = gm;

	// 测试偏航角度
	cv::Point3f test_P;
	test_P.x = gm.yaw;

	// 输出预测完成的提示信息
	{
		std::cout << GREENCOLOR << "predict finished" << std::endl;
		std::cout << WHITECOLOR << std::endl;
	}

return gm;
}
//*****************************Track end*************************************** */


/**
 * @brief:  返回非预测装甲板位置，衍生自预测程序
 *
 * @author: zyl
 *
 * @param:  imu_data是IMU姿态解算的结果, objects是本帧所有的装甲板，time是图像的时间戳
 *
 * @return: 当前装甲板的云台上台和偏航的角度
 */
GimbalPose PredictorPose::run_current(GimbalPose &imu_data, std::vector<DetectObject> &objects, double time)
{
	bool terminal_flag = false; //终端是否显示输出内容

	imu_data_ = imu_data;
	current_time_ = time;

	Eigen::Matrix3d pitch_rotation_matrix_t;
	pitch_rotation_matrix_t
		<< 1,
		0, 0,
		0, std::cos((imu_data_.pitch * CV_PI) / 180), std::sin((imu_data_.pitch * CV_PI) / 180),
		0, -std::sin((imu_data_.pitch * CV_PI / 180)), std::cos((imu_data_.pitch * CV_PI) / 180);

	Eigen::Matrix3d yaw_rotation_matrix_t;
	yaw_rotation_matrix_t
		<< std::cos((imu_data_.yaw * CV_PI) / 180),
		0, std::sin((imu_data_.yaw * CV_PI) / 180),
		0, 1, 0,
		-std::sin((imu_data_.yaw * CV_PI) / 180), 0, std::cos((imu_data_.yaw * CV_PI) / 180);

	Eigen::Matrix3d transform_vector;
	transform_vector =  yaw_rotation_matrix_t * pitch_rotation_matrix_t;
	transform_vector_ = transform_vector.inverse();
	// 如果这一帧没有装甲板，先看看init_是true还是false
	// 是true

if (!objects.size())
{
    // 增加丢失计数器，并限制其最大值为250
    loss_cnt_++;
    if (loss_cnt_ > loss_cnt_max)
    {
        loss_cnt_ = loss_cnt_max;
    }

    // 如果当前状态已经是LOSS状态，则直接返回最后一帧的云台角度
    if (state_ == ARMOR_STATE_::LOSS)
    {
        find_switch = false;
        return last_eular_;
    }

    // 如果丢失计数器超过cru_los_maxq，则切换到LOSS状态，并返回最后一帧的云台角度
    if (loss_cnt_ > cru_los_maxq)
    {
        state_ = ARMOR_STATE_::LOSS;
        last_state_ = state_;
        // 返回最后一帧的云台角，避免剧烈晃动
        init_ = true;
        find_switch = false;
        velocities_.clear();
        yaw_rad_.clear();
        time_buff_.clear();
		// std::cout << RED1 << "LOSS ARMOR!!!" << std::endl;
		// std::cout << WHITE << std::endl;
        return last_eular_;
    }

    // 如果丢失计数器未超过cru_los_maxq，则保持TRACK状态，并进行目标位置预测
    state_ = ARMOR_STATE_::TRACK;
    last_state_ = state_;

    // 根据上一帧的位置和速度预测当前帧的目标位置
    Eigen::Vector3d current_pose;
    current_pose[0] = last_location_[0] + last_velocity_[0] * (current_time_ - last_time_);
    current_pose[1] = last_location_[1] + last_velocity_[1] * (current_time_ - last_time_);
    current_pose[2] = last_location_[2] + last_velocity_[2] * (current_time_ - last_time_);
    float fly_t = bullteFlyTime(current_pose);

    // 根据预测的飞行时间进一步预测目标位置
    Eigen::Vector3d predict_pose;
    predict_pose[0] = current_pose[0] + last_velocity_[0] * fly_t;
    predict_pose[1] = current_pose[1] + last_velocity_[1] * fly_t;
    predict_pose[2] = current_pose[2] + last_velocity_[2] * fly_t;

    // 根据不同的条件更新预测位置和飞行时间
    bullteFlyTime(current_pose);
    last_location_ = current_pose;
    predict_location_ = predict_pose;

    // 更新云台角度并返回
    GimbalPose gm = gm_ptz;
    last_eular_ = gm;

    return gm;
}

	loss_cnt_ = 0;
	if (init_)
	{
		//std::cout << "init finished!!!" << std::endl;
		init();
		state_ = ARMOR_STATE_::TRACK;
		last_state_ = state_;
		last_obj_nums = objects.size();
		// 选择一个装甲板
		float distances[objects.size()];
		for (int i = 0; i < objects.size(); i++)
		{
			cv::Point2f center = (objects[i].pts[0] + objects[i].pts[2]) / 2;
			distances[i] = (center.x - 512) + (center.y - 640);
		}

		int index = 0;
		for (int i = 1; i < objects.size(); i++)
		{
			if (distances[i] < distances[index])
				index = i;
		}
		DetectObject obj = objects[index];

		std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
		world_cam_pose = pnp_solve_->poseCalculation(obj,terminal_flag);

		std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;

		cam_ptz_pose.first = cam3ptz(imu_data_, world_cam_pose.first,terminal_flag);
		cam_ptz_pose.second = world_cam_pose.second;

		// std::cout << "[world x]: " << cam_ptz_pose.first[0] << "  "
		// 		  << "[world y]: " << cam_ptz_pose.first[1] << "  [world z]: " << cam_ptz_pose.first[2] << std::endl;

		last_pose_ = cam_ptz_pose; // 记录这一时刻，为下一时刻预测做准备
		init_ = false;
		last_location_ = cam_ptz_pose.first;
		// !!! return
		float fly_t = bullteFlyTime(cam_ptz_pose.first);
		last_time_ = current_time_;
		GimbalPose gm = gm_ptz;
		velocities_.clear();
		last_id = obj.label;
		last_obj = obj;
		last_eular_ = gm;
		return gm;
	}

	//std::cout << "continue track armor!!!" << std::endl;

	// 选择一个装甲板
	// 当卡方检验过大时仍然要打开初始化开关
	state_ = ARMOR_STATE_::TRACK;
	DetectObject obj = ArmorSelect(objects,terminal_flag);
	obj_pixe_ = (obj.pts[0] + obj.pts[2]) / 2;
	// world_cam_pose = pnp_solve_->poseCalculation(obj);

	std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;

	float distance_;
	distance_ = std::sqrt(obj.coord[0] * obj.coord[0] + obj.coord[2] * obj.coord[2]);
	float yaw_now = std::asin(obj.coord[0] / distance_) * 180 / CV_PI;

	float distance_last;
	distance_last = std::sqrt(last_obj.coord[0] * last_obj.coord[0] + last_obj.coord[2] * last_obj.coord[2]);
	float yaw_last = std::asin(last_obj.coord[0] / distance_last) * 180 / CV_PI;
	
    time_diff = current_time_ - last_switch_time_;

	cam_ptz_pose.first = obj.coord;
	current_location_ = obj.coord;//by wty（此为理论上的白点，存在问题）
	cam_ptz_pose.second = obj.pyr;
	

	kf.is_change_direction_ = switch_flag;
	cam_ptz_pose.first = kf.KalmanMainRun(cam_ptz_pose.first,last_velocity_);

	//std::cout << "world coord solve finished" << std::endl;

	float fly_t = bullteFlyTime(cam_ptz_pose.first);

	//std::cout << "the first fly time geted" << std::endl;

	Eigen::Vector4d current_state; // 现在的速度: 直接计算的速度，未经过拟合
	current_state[0] = cam_ptz_pose.first[0];
	current_state[1] = cam_ptz_pose.first[1];
	current_state[2] = cam_ptz_pose.first[2];
	current_state[3] = current_time_;

	if (velocities_.size() < velocities_deque_size_)
	{
		velocities_.push_back(current_state);
	}
	else
	{
		velocities_.pop_front();
		velocities_.push_back(current_state);
	}

	Eigen::Vector3d now_v = CeresVelocity(velocities_);

	if (isnan(now_v[0]) || isnan(now_v[1]) || isnan(now_v[2]))
	{
		now_v = last_velocity_;
	}


	//now_v[0] = 0;//一般就是注释上的，解除可能导致红点斜向来会折返
	now_v[1] = 0;
	//now_v[2] = 0;
	cv::Point3f point;
	point.x = now_v[0];


	Eigen::Vector3d predict_location;

	//std::cout << "[ vx: " << now_v[0] << " vy: " << now_v[1] << " vz: " << now_v[2] << " ]" << std::endl;

	predict_location[0] = cam_ptz_pose.first[0] + (now_v[0] * (fly_t));
	predict_location[1] = cam_ptz_pose.first[1] + (now_v[1] * (fly_t));
	predict_location[2] = cam_ptz_pose.first[2] + (now_v[2] * (fly_t));


	//std::cout << "predict expression " << (now_v[0] * (fly_t)) * 100 << " cm " << std::endl;
	move_ = (now_v[0] * (fly_t)) * 100;

	bullteFlyTime(current_location_);//启用时将发送白点的p和y值


	last_location_ = cam_ptz_pose.first;
	last_time_ = current_time_;
	last_velocity_ = now_v;
	last_state_ = state_;
	last_pose_ = cam_ptz_pose;
	predict_location_ = predict_location;

    //测试击打装甲板时根据弹道偏向修改这里的p和y加减的数值，单位为弧度值
	GimbalPose gm = gm_ptz;
	gm.yaw = gm.yaw + yaw_corangle;
	gm.pitch = gm.pitch + pit_corangle;
	last_obj = obj;
	last_eular_ = gm;

	cv::Point3f test_P;
	test_P.x = gm.yaw;

	return gm;
}

/**
 * @brief 弹道解算，计算子弹飞行时间
 * 
 * 该函数根据给定的目标坐标，计算子弹从当前位置飞行到目标位置所需的时间。
 * 计算过程中考虑了子弹的初速度、重力加速度以及目标的yaw和pitch角度。
 * 
 * @param coord 目标三维坐标，类型为Eigen::Vector3d，包含x, y, z三个分量
 * @return float 返回子弹飞行时间，如果计算失败则返回-1
 */
float PredictorPose::bullteFlyTime(Eigen::Vector3d coord)
{
    // 将Eigen::Vector3d坐标转换为cv::Point3f格式
    cv::Point3f p1;
    p1.x = coord[0];
    p1.y = coord[1];
    p1.z = coord[2];


    // 打印目标坐标
    std::cout << "[p1.x] " << p1.x << " [p1.y] " << p1.y << " [p1.z] " << p1.z << std::endl;

    // 调整子弹初速度
    float v0_ = v0 - 0.5;
    if (v0_ == 0)
    {
        v0_ = 30;
    }
    // 打印调整后的子弹速度
    std::cout << "bullet speed is: " << v0_ << std::endl;

    // 重力加速度
    float g = 9.80665;

    // 计算目标的yaw角度
    float distance1 = std::sqrt(p1.x * p1.x + p1.z * p1.z);
    gm_ptz.yaw = std::asin(p1.x / distance1) * 180 / CV_PI;


    // 根据目标坐标的象限调整yaw角度
    if (p1.z < 0 && p1.x < 0)
    {
        gm_ptz.yaw = gm_ptz.yaw - 2 * (gm_ptz.yaw + 90);
    }
    if (p1.z < 0 && p1.x > 0)
    {
        gm_ptz.yaw = gm_ptz.yaw + 2 * (90 - gm_ptz.yaw);
    }

    // 打印计算得到的yaw角度
    std::cout << "[yaw: ]" << gm_ptz.yaw << std::endl;


    // 计算目标的pitch角度
    float a = -0.5 * g * (std::pow(distance1, 2) / std::pow(v0_, 2));
    float b = distance1;
    float c = a - p1.y;

    // 计算判别式，判断是否有实数解
    float Discriminant = std::pow(b, 2) - 4 * a * c;
    if (Discriminant < 0)
        return -1;

    // 计算两个可能的pitch角度
    float tan_angle1 = (-b + std::pow(Discriminant, 0.5)) / (2 * a);
    float tan_angle2 = (-b - std::pow(Discriminant, 0.5)) / (2 * a);

    // 将pitch角度转换为角度制
    float angle1 = std::atan(tan_angle1) * 180 / CV_PI;
    float angle2 = std::atan(tan_angle2) * 180 / CV_PI;

    // 选择合理的pitch角度
    if (tan_angle1 >= -3 && tan_angle1 <= 3)
    {
        gm_ptz.pitch = angle1;
    }
    else if (tan_angle2 >= -3 && tan_angle2 <= 3)
    {
        gm_ptz.pitch = angle2;
    }

    // 将pitch角度转换为弧度制
    float PI_pitch = (gm_ptz.pitch / 180) * CV_PI;

    // 打印计算得到的pitch角度
    std::cout << "[pitch: ]" << gm_ptz.pitch << std::endl;

    // 计算并返回子弹飞行时间
    return distance1 / (v0_ * std::cos(PI_pitch));
}

/**
 * @brief 从一组检测到的物体中选择一个最合适的装甲板目标。
 * 
 * 该函数通过计算每个物体在相机坐标系中的位置，并将其转换为PTZ坐标系，
 * 然后根据与上一次选择的目标位置的距离残差，选择最接近的目标。
 * 
 * @param objects 包含所有检测到的物体的向量，每个物体包含其坐标和姿态信息。
 * @return DetectObject 返回选择的最合适的装甲板目标。
 */
DetectObject PredictorPose::ArmorSelect(std::vector<DetectObject> &objects,bool t_flag)
{
    // 遍历所有检测到的物体，计算其在世界坐标系和相机坐标系中的位置
    for (int i = 0; i < objects.size(); i++)
    {
        // 计算物体在世界坐标系中的位置和姿态
        std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
        world_cam_pose = pnp_solve_->poseCalculation(objects[i],t_flag);

        // 将世界坐标系中的位置转换为PTZ坐标系中的位置
        std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
        cam_ptz_pose.first = cam3ptz(imu_data_, world_cam_pose.first,t_flag);
        cam_ptz_pose.second = world_cam_pose.second;

        // 更新物体的坐标和姿态信息
        objects[i].coord = cam_ptz_pose.first;
        objects[i].pyr = cam_ptz_pose.second;
    }

    // 计算每个物体到原点的距离
    float distances[objects.size()];
    for (int i = 0; i < objects.size(); i++)
    {
        distances[i] = std::sqrt(std::pow(objects[i].coord[0], 2) + std::pow(objects[i].coord[1], 2) + std::pow(objects[i].coord[2], 2));
    }

    // 计算上一次选择的目标位置到原点的距离
    float last_pose = std::sqrt(std::pow(last_pose_.first[0], 2) + std::pow(last_pose_.first[1], 2) + std::pow(last_pose_.first[2], 2));

    // 计算每个物体与上一次选择的目标位置的距离残差
    float distances_residual[objects.size()];
    for (int i = 1; i < objects.size(); i++)
    {
        distances_residual[i] = std::abs(distances[i] - last_pose);
    }
 
    // 选择距离残差最小的物体作为目标
    int index = 0;
    for (int i = 1; i < objects.size(); i++)
    {
        if (distances_residual[i] < distances_residual[index])
            index = i;
    }

    // 返回选择的目标
    return objects[index];
}
#endif
#ifdef NT
/**
 * @brief  判断弹道的飞行时间
 *
 * @param  三维坐标
 *
 * @return 飞行的时间
 */
double PredictorPose::bullteFlyTime(Eigen::Vector3d coord)
{
    cv::Point3f p1;

    p1.x = coord[0];
    p1.y = coord[1];
    p1.z = coord[2];

    float g = 9.8065;

    // 先算yaw的值的
    float distance1;
    distance1 = std::sqrt(p1.x * p1.x + p1.z * p1.z);
    gm_ptz.yaw = std::asin(p1.x / distance1) * 180 / CV_PI;

    if (p1.z < 0 && p1.x >= 0)
    {
        gm_ptz.yaw = 2 * (90 - gm_ptz.yaw) + gm_ptz.yaw;
    }
    else if (p1.z < 0 && p1.x < 0)
    {
        gm_ptz.yaw = 2 * (-90 - gm_ptz.yaw) + gm_ptz.yaw;
    }

    // pitch值
    double a = -0.5 * g * (std::pow(distance1, 2) / std::pow(v0, 2));
    double b = distance1;
    double c = a - p1.y;

    double Discriminant = std::pow(b, 2) - 4 * a * c; // 判别式
    if (Discriminant < 0)
        return -1;
    double tan_angle1 = (-b + std::pow(Discriminant, 0.5)) / (2 * a); //*180/CV_PI;
    double tan_angle2 = (-b - std::pow(Discriminant, 0.5)) / (2 * a);

    float angle1 = std::atan(tan_angle1) * 180 / CV_PI; // 角度制
    float angle2 = std::atan(tan_angle2) * 180 / CV_PI; // 角度制

    if (tan_angle1 >= -3 && tan_angle1 <= 3)
    {
        gm_ptz.pitch = angle1;
    }
    else if (tan_angle2 >= -3 && tan_angle2 <= 3)
    {
        gm_ptz.pitch = angle2;
    }

    double PI_pitch = (gm_ptz.pitch / 180) * CV_PI; // 弧度制

    return distance1 / (v0 * std::cos(PI_pitch));
}
DetectObject PredictorPose::ArmorSelect(std::vector<DetectObject> objects)
{
    double distances[objects.size()];
    double last_pose_distance = std::sqrt(
        std::pow(last_pose_.first[0], 2) + std::pow(last_pose_.first[1], 2) + std::pow(last_pose_.first[2], 2));
    double distances_residual[objects.size()];
    for (unsigned int i = 0; i < objects.size(); i++)
    {
        std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose = pnp_solve_->poseCalculation(objects[i]);
        std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose = cam2ptz(world_cam_pose, imu_data_);

        distances[i] = std::sqrt(
            std::pow(cam_ptz_pose.first[0], 2) + std::pow(cam_ptz_pose.first[1], 2) +
            std::pow(cam_ptz_pose.first[2], 2));
        distances_residual[i] = std::abs(distances[i] - last_pose_distance);

        if ((objects[i].label == 1 || objects[i].label == 10) && distances[i] < 3)
        { // 选择英雄
            std::cout << "ChoosedHero" << std::endl;
            return objects[i];
        }
    }

    unsigned int index = 0;
    for (unsigned int i = 1; i < objects.size(); i++)
    {
        if (distances_residual[i] < distances_residual[index])
            index = i;
    }

    return objects[index];
}
#endif

/**
 * @brief 使用最小二乘法拟合速度，并返回预测的速度向量。
 * 
 * 该函数通过最小二乘法对给定的速度队列进行拟合，计算出速度的线性趋势。
 * 如果速度队列的大小小于4，则返回上一次计算的速度。
 * 如果定义了Nova_CeresVelocity宏，则会对拟合后的速度进行移动窗口平均滤波处理。
 * 
 * @param velocities 包含速度和时间戳的队列，每个元素是一个Eigen::Vector4d，其中前三个元素是速度分量，第四个元素是时间戳。
 * @return Eigen::Vector3d 返回拟合后的速度向量，包含x、y、z三个方向的速度分量。
 */
Eigen::Vector3d PredictorPose::CeresVelocity(std::deque<Eigen::Vector4d> velocities)
{
    unsigned int N = velocities.size();
    if (N < 4)
    {
        return last_velocity_;
    }

    // 计算x方向速度的拟合参数
    double avg_x = 0;
    double avg_x2 = 0;
    double avg_f = 0;
    double avg_xf = 0;

    double time_first = velocities.front()[3];

    for (unsigned int i = 0; i < N; i++)
    {
        avg_x += velocities[i][3] - time_first;
        avg_x2 += std::pow(velocities[i][3] - time_first, 2);
        avg_f += velocities[i][0];
        avg_xf += (velocities[i][3] - time_first) * velocities[i][0];
    }

    double vx = (avg_xf - N * (avg_x / N) * (avg_f / N)) / (avg_x2 - N * std::pow(avg_x / N, 2));

    // 计算y方向速度的拟合参数
    avg_x = 0;
    avg_x2 = 0;
    avg_f = 0;
    avg_xf = 0;
    for (unsigned int i = 0; i < N; i++)
    {
        avg_x += velocities[i][3] - time_first;
        avg_x2 += std::pow(velocities[i][3] - time_first, 2);
        avg_f += velocities[i][1];
        avg_xf += (velocities[i][3] - time_first) * velocities[i][1];
    }
    double vy = (avg_xf - N * (avg_x / N) * (avg_f / N)) / (avg_x2 - N * std::pow(avg_x / N, 2));

    // 计算z方向速度的拟合参数
    double avg_x_ = 0;
    double avg_x2_ = 0;
    double avg_f_ = 0;
    double avg_xf_ = 0;
    for (unsigned int i = 0; i < N; i++)
    {
        avg_x_ += velocities[i][3] - time_first;
        avg_x2_ += std::pow(velocities[i][3] - time_first, 2);
        avg_f_ += velocities[i][2];
        avg_xf_ += (velocities[i][3] - time_first) * velocities[i][2];
    }
    double vz = (avg_xf_ - N * (avg_x_ / N) * (avg_f_ / N)) / (avg_x2_ - N * std::pow(avg_x_ / N, 2));

#ifndef NT
#ifdef Nova_CeresVelocity
    // 添加移动窗口平均滤波器
    const int windowSize = 4; // 窗口大小
    static std::deque<Eigen::Vector3d> velocityBuffer;
    velocityBuffer.push_back(Eigen::Vector3d(vx, vy, vz));
    if (velocityBuffer.size() > windowSize)
    {
        velocityBuffer.pop_front();
    }

    // 去掉窗口内的最大值和最小值
    std::vector<Eigen::Vector3d> sortedVelocities(velocityBuffer.begin(), velocityBuffer.end());
    std::sort(sortedVelocities.begin(), sortedVelocities.end(), [](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
              { return a.norm() < b.norm(); });

    Eigen::Vector3d filteredVelocity(0, 0, 0);
    for (size_t i = 1; i < sortedVelocities.size() - 1; i++)
    {
        filteredVelocity += sortedVelocities[i];
    }
    filteredVelocity /= (sortedVelocities.size() - 2);

    return filteredVelocity;
#endif

#ifndef Nova_CeresVelocity
    // 如果x方向速度与上一次计算的速度方向相反且v_count小于4，则保持上一次的速度
    if (vx * last_velocity_[0] < 0 && v_count < 4)
    {
        vx = last_velocity_[0];
        v_count++;
    }
    else
    {
        v_count = 0;
    }

    return {vx, vy, vz};
#endif
#endif
#ifdef NT

    // 更新指数加权移动平均值
    ema_velocity_ = smoothing_factor * Eigen::Vector3d(vx, vy, vz) + (1 - smoothing_factor) * ema_velocity_;

    // 使用ema_velocity_作为最终返回的速度值
    return ema_velocity_;
#endif
}

#ifndef NT
/**
 * @brief 根据云台姿态和相机位置，计算相机在世界坐标系中的位置。
 * 
 * 该函数通过给定的云台姿态（俯仰角和偏航角）和相机在云台坐标系中的位置，
 * 计算相机在世界坐标系中的位置。首先对相机位置进行偏置校正，然后根据云台
 * 的俯仰角和偏航角构建旋转矩阵，最后将相机位置转换到世界坐标系中。
 * 
 * @param gm 云台姿态，包含俯仰角（pitch）和偏航角（yaw）。
 * @param pos 相机在云台坐标系中的位置，是一个三维向量。
 * @return Eigen::Vector3d 相机在世界坐标系中的位置，是一个三维向量。
 */
Eigen::Vector3d PredictorPose::cam3ptz(GimbalPose gm, Vector3d pos,bool t_flag)
{
    // 对相机位置进行偏置校正
    pos[0] = pos[0] + X_BIAS;
    pos[1] = pos[1] + Y_BIAS;
    pos[2] = pos[2] + Z_BIAS;
	// pos = pos.transpose();

    // 定义俯仰角和偏航角的旋转矩阵
    Matrix3d pitch_rotation_matrix_;
    Matrix3d yaw_rotation_matrix_;

    // 打印相机位置和云台姿态信息
	if(t_flag == true)
	{
    std::cout << "camera pose " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    std::cout << "solve gm.pitch" << gm.pitch << std::endl;
    std::cout << "solve gm.yaw" << gm.yaw << std::endl;
	}
	
    // 构建俯仰角的旋转矩阵
    pitch_rotation_matrix_
        << 1,
        0, 0,
        0, std::cos((gm.pitch) * (CV_PI / 180)), std::sin((gm.pitch) * (CV_PI / 180)),
        0, -std::sin((gm.pitch) * (CV_PI / 180)), std::cos((gm.pitch) * (CV_PI / 180));

    // 构建偏航角的旋转矩阵
    yaw_rotation_matrix_
        << std::cos((gm.yaw) * (CV_PI / 180)),
        0, std::sin((gm.yaw) * (CV_PI / 180)),
        0, 1, 0,
        -std::sin((gm.yaw) * (CV_PI / 180)), 0, std::cos((gm.yaw) * (CV_PI / 180));

    // 将相机位置转换到世界坐标系中
    Vector3d t_pos_ =  yaw_rotation_matrix_ * pitch_rotation_matrix_ * pos;

    // 打印转换后的相机位置
	if(t_flag == true)
	{
    std::cout << "world pose " << t_pos_[0] << " " << t_pos_[1] << " " << t_pos_[2] << std::endl;
	//std::cout << "tf = true" << std::endl;
	}
    return t_pos_;
}

/**
 * @brief 判断是否发生了姿态切换
 * 
 * 该函数通过计算两个三维向量的偏航角（yaw）差异，判断是否发生了姿态切换。
 * 
 * @param up_switch 当前时刻的三维向量，表示当前姿态
 * @param down_switch 上一时刻的三维向量，表示上一时刻的姿态
 * @return bool 如果偏航角差异大于3.0度，返回true，表示发生了姿态切换；否则返回false
 */
bool PredictorPose::isSwitch(Vector3d up_switch, Vector3d down_switch)
{
    // 计算当前时刻向量的水平距离（忽略Y轴）
    float distance_ = std::sqrt(up_switch[0] * up_switch[0] + up_switch[2] * up_switch[2]);
    // 计算当前时刻的偏航角（yaw），并将其转换为角度制
    float yaw_now = std::asin(up_switch[0] / distance_) * 180 / CV_PI;

    // 计算上一时刻向量的水平距离（忽略Y轴）
    float distance_last = std::sqrt(down_switch[0] * down_switch[0] + down_switch[2] * down_switch[2]);
    // 计算上一时刻的偏航角（yaw），并将其转换为角度制
    float yaw_last = std::asin(down_switch[0] / distance_last) * 180 / CV_PI;

    // 判断当前时刻与上一时刻的偏航角差异是否大于3.0度
    if (std::abs(yaw_now - yaw_last) > 3.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

#endif

#ifdef Best_Yaw
double PredictorPose::best_yaw(TRTInferV1::DetectObject obj)
{
    // PnP 算法得到的平移向量 tvec
    Eigen::Vector3d tvec = obj.coord;
    double width = 0.135, height = 0.057; // 目标长宽
    std::vector<cv::Point2f> boundaryPoints;
    for (int i = 0; i < 4; i++)
    {
        boundaryPoints.push_back(obj.pts[i]); // 像素坐标系 目标边界点坐标顺序 2 3 4 1
    }
    cv::Mat cameraIntrinsics = pnp_solve_->K_;                      // 相机内参矩阵
    Eigen::Vector3d turretToCamTranslation(X_BIAS, Y_BIAS, Z_BIAS); // 云台坐标系到相机坐标系的平移向量
    double bestYaw = goldenSectionSearch(-M_PI / 3, M_PI / 3, tvec, width, height, boundaryPoints, cameraIntrinsics,
                                         turretToCamTranslation);
    // 计算最优解下的目标四个顶点坐标
    std::vector<Eigen::Vector3d> targetPoints = calculateTargetPoints(tvec, width, height, bestYaw);
    std::vector<cv::Point2f> projPoints = projectPoints(targetPoints, cameraIntrinsics, turretToCamTranslation);

    for (const auto &projPoint : projPoints)
    {
        std::cout << "\033[34m" << projPoint << "\033[0m" << std::endl;
    }
    toVofa.bestyaw = (float)(bestYaw * 180) / M_PI;
    std::cout << "\033[34m" << (bestYaw * 180) / M_PI << "\033[0m" << std::endl;
    return bestYaw;
}
// 计算云台系下四个点的坐标
std::vector<Eigen::Vector3d>
PredictorPose::calculateTargetPoints(const Eigen::Vector3d &center, double width, double height, double yaw)
{
    double pitch = 15 * M_PI / 180.0; // pitch已知为15度
    // 计算旋转矩阵
    Matrix3d Rx;
    Rx << 1, 0, 0,
        0, cos(pitch), -sin(pitch),
        0, sin(pitch), cos(pitch);

    Matrix3d Ry;
    Ry << cos(yaw), 0, sin(yaw),
        0, 1, 0,
        -sin(yaw), 0, cos(yaw);

    Matrix3d R = Ry * Rx; // 注意旋转顺序

    // 计算局部坐标系下的四个顶点
    Vector3d p1(width / 2, height / 2, 0);   // 右上
    Vector3d p2(width / 2, -height / 2, 0);  // 右下
    Vector3d p3(-width / 2, height / 2, 0);  // 左上
    Vector3d p4(-width / 2, -height / 2, 0); // 左下
    std::vector<Eigen::Vector3d> vertices(4);
    // 旋转并平移到全局坐标系
    // 按照左上、左下、右下、右上的顺序存储四个顶点
    vertices[3] = R * p1 + center;
    vertices[2] = R * p2 + center;
    vertices[0] = R * p3 + center;
    vertices[1] = R * p4 + center;
    return vertices;
}

// 将四个点重投影到图像坐标系
std::vector<cv::Point2f>
PredictorPose::projectPoints(const std::vector<Eigen::Vector3d> &points3d, const cv::Mat &cameraIntrinsics,
                             const Eigen::Vector3d &turretToCamTranslation)
{
    std::vector<cv::Point2f> projPoints;
    // 左上、左下、右下、右上的顺序进行投影
    projPoints.reserve(points3d.size()); // 预先分配足够的空间
    for (const auto &i : points3d)
    {
        // 将点从世界坐标系变换到相机坐标系
        Eigen::Vector3d camPoint = (transform_vector_)*i;
        camPoint -= turretToCamTranslation;
        camPoint[1] = -camPoint[1]; // 对Y坐标进行镜像变换
        Eigen::Matrix3d F;
        cv2eigen(cameraIntrinsics, F);
        // 相机坐标系内坐标--->图像坐标系内像素坐标
        Eigen::Vector3d pu = F * camPoint / camPoint.z();
        // 转换为像素坐标系中的点
        cv::Point2f imgP(pu.x(), pu.y());
        projPoints.push_back(imgP);
    }
    return projPoints;
}

// 代价函数
double PredictorPose::objectiveFunc(double yaw, const Eigen::Vector3d &center, double width, double height,
                                    const std::vector<cv::Point2f> &boundaryPoints, const cv::Mat &cameraIntrinsics,
                                    const Eigen::Vector3d &turretToCamTranslation)
{
    std::vector<Eigen::Vector3d> targetPoints = calculateTargetPoints(center, width, height, yaw);
    std::vector<cv::Point2f> projPoints = projectPoints(targetPoints, cameraIntrinsics, turretToCamTranslation);

    double sumDist = 0;
    for (unsigned int i = 0; i < projPoints.size(); i++)
    {
        sumDist += std::pow(projPoints[i].x - boundaryPoints[i].x,2) + std::pow(projPoints[i].y - boundaryPoints[i].y,2);
    }

    double lengthPro = std::sqrt(std::pow(projPoints[0].x - projPoints[3].x, 2) + std::pow(projPoints[0].y - projPoints[0].y, 2)) +
                       std::sqrt(std::pow(projPoints[1].x - projPoints[2].x, 2) + std::pow(projPoints[1].y - projPoints[2].y, 2));
    double lengthBound = std::sqrt(std::pow(boundaryPoints[0].x - boundaryPoints[3].x, 2) + std::pow(boundaryPoints[0].y - boundaryPoints[0].y, 2)) +
                         std::sqrt(std::pow(boundaryPoints[1].x - boundaryPoints[2].x, 2) + std::pow(boundaryPoints[1].y - boundaryPoints[2].y, 2));
    double AreaPro = std::sqrt(std::pow(projPoints[0].x - projPoints[1].x, 2) + std::pow(projPoints[0].y - projPoints[1].y, 2)) *
                     std::sqrt(std::pow(projPoints[1].x - projPoints[2].x, 2) + std::pow(projPoints[1].y - projPoints[2].y, 2));
    double AreaBound = std::sqrt(std::pow(boundaryPoints[0].x - boundaryPoints[1].x, 2) + std::pow(boundaryPoints[0].y - boundaryPoints[1].y, 2)) *
                       std::sqrt(std::pow(boundaryPoints[1].x - boundaryPoints[2].x, 2) + std::pow(boundaryPoints[1].y - boundaryPoints[2].y, 2));
    sumDist += (lengthPro - lengthBound) + (AreaPro - AreaBound);
    
    toVofa.CurrentYaw = (float)(yaw * 180) / M_PI;
    toVofa.sumdist = (float)sumDist;
    return sumDist;
}

// 优选法三分法
double PredictorPose::goldenSectionSearch(double a, double b, const Eigen::Vector3d &center, double width, double height,
                                          const std::vector<cv::Point2f> &boundaryPoints,
                                          const cv::Mat &cameraIntrinsics,
                                          const Eigen::Vector3d &turretToCamTranslation)
{
     while (std::abs(b - a) > 1e-8) {
        double m1 = a + (b - a) / 3.0;
        double m2 = b - (b - a) / 3.0;

        double f1 = objectiveFunc(m1, center, width, height, boundaryPoints, cameraIntrinsics, turretToCamTranslation);
        double f2 = objectiveFunc(m2, center, width, height, boundaryPoints, cameraIntrinsics, turretToCamTranslation);

        if (f1 < f2) {
            b = m2;
        } else {
            a = m1;
        }
    }

    return (a + b) / 2.0;
}

#endif

// #ifdef NT
// /**
//  * @brief 绘制x轴的波形图
//  * @param  point
//  * @author AlexLi
//  * @date 2024-03-08
//  */
// void drawCurveData(cv::Point3f point)
// {

//     using namespace cv;
//     Mat poly_background_src_ = cv::Mat::zeros(640, 512, CV_8UC3);
//     poly_background_src_.setTo(cv::Scalar(0, 255, 0));

//     SavePoint[Times % SIN_POINT_NUM] = point.x * 100;

//     char test[100];
//     sprintf(test, "vx:%0.4f", point.x * 100);
//     cv::putText(poly_background_src_, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1,
//                 8);

//     if (point.x * 100 > 30)
//     {
//         sprintf(test, " vx large:%s ", "true");
//     }
//     else
//     {
//         sprintf(test, " vx large:%s ", "false");
//     }
//     cv::putText(poly_background_src_, test, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1,
//                 8);

//     // 找最大值的方法只用循环一次就可以
//     float maxData = 0;
//     for (int i = 0; i <= Times && i < SIN_POINT_NUM; i++)
//     {
//         if (fabs(SavePoint[i]) > maxData)
//         {
//             maxData = fabs(SavePoint[i]);
//         }
//     }
//     // 计算倍率
//     float bei = 10;
//     if (10 * maxData > poly_background_src_.rows / 2)
//     {
//         bei = (poly_background_src_.rows / 2 - 10) / maxData;
//     }
//     if (10 * maxData < 150 && maxData != 0)
//     {
//         bei = 150.0 / maxData;
//     }
//     std::vector<cv::Point> points;
//     // 存入当前记录位置的后续单位,靠前的位置
//     int t = 0;
//     for (int i = (Times + 1) % SIN_POINT_NUM; i <= Times && i < SIN_POINT_NUM; i++)
//     {
//         points.push_back(Point((float)poly_background_src_.cols / SIN_POINT_NUM * t,
//                                poly_background_src_.rows / 2 + bei * SavePoint[i]));
//         t++;
//     }
//     // 绘制折线
//     // cv::polylines(poly_background_src_, points, false, cv::Scalar(255, 0, 0), 1, 8, 0);
//     // 存入之前的元素
//     for (int i = 0; i <= Times % SIN_POINT_NUM; i++)
//     {
//         points.emplace_back((float)poly_background_src_.cols / SIN_POINT_NUM * t,
//                             poly_background_src_.rows / 2 + bei * SavePoint[i]);
//         t++;
//     }

//     cv::polylines(poly_background_src_, points, false, cv::Scalar(0, 0, 255), 3, 8, 0);

//     // cv::polylines(poly_background_src_, points, false, cv::Scalar(255, 0, 0), 3, 8, 0);

//     string windowName = "波形图-预测前";
//     namedWindow(windowName, 0);
//     imshow(windowName, poly_background_src_);
//     Times++;
// }
// #endif
// /**
//  * @brief 将旋转矩阵转化为欧拉角
//  * @param R 旋转矩阵
//  * @return 欧拉角
//  */
// Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R) //by lqq已废弃
// {
// 	double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
// 	bool singular = sy < 1e-6;
// 	double x, y, z;
// 	if (!singular)
// 	{
// 		x = atan2(R(2, 1), R(2, 2));
// 		y = atan2(-R(2, 0), sy);
// 		z = atan2(R(1, 0), R(0, 0));
// 	}
// 	else
// 	{
// 		x = atan2(-R(1, 2), R(1, 1));
// 		y = atan2(-R(2, 0), sy);
// 		z = 0;
// 	}
// 	return {z, y, x};
// }

// DetectObject PredictorPose::ArmorChoice(std::vector<DetectObject> &objects) //by wty 未测试
// {
//     double distances[objects.size()];
//     double last_pose_distance = std::sqrt(std::pow(last_pose_.first[0], 2) + std::pow(last_pose_.first[1], 2) + std::pow(last_pose_.first[2], 2));
//     double distances_residual[objects.size()];
//     if(objects.size() == 0)
//     {
//         return last_obj;
//     }
//     for (unsigned int i = 0; i < objects.size(); i++)
//     {
//         std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose = pnp_solve_->poseCalculation(objects[i],terminal_flag);
//         std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
// 		cam_ptz_pose.first = cam3ptz(imu_data_ , world_cam_pose.first);
// 		cam_ptz_pose.second = world_cam_pose.second;

//         distances[i] = std::sqrt(
//             std::pow(cam_ptz_pose.first[0], 2) + std::pow(cam_ptz_pose.first[1], 2) +
//             std::pow(cam_ptz_pose.first[2], 2));
//         distances_residual[i] = std::abs(distances[i] - last_pose_distance);
// 		//近距离英雄优先级最高
//         if ((objects[i].label == 1 || objects[i].label == 10) && distances[i] < 3)
//         { // 选择英雄
//             std::cout << "ChoosedHero" << std::endl;
//             return objects[i];
//         }
//     }
// 	//寻找距离最近装甲板
//     unsigned int index = 0;
//     for (unsigned int i = 1; i < objects.size(); i++)
//     {
//         if (distances_residual[i] < distances_residual[index])
//             index = i;
//     }
// 	//判断是否一个车上有两个装甲板被识别
//     int DoubleArrmor = 0;
//     std::vector<DetectObject> same_id_armors;
//     for (int i = 0; i < objects.size(); i++)
//     {
//         if (objects[i].label == objects[index].label)
//         {
//             DoubleArrmor++;
//             same_id_armors.push_back(objects[i]);
//         }
//     }
// 	//若只有一个相同id装甲板取距离最近的装甲板
//     if(same_id_armors.size() == 1)
//     {
//         return objects[index];
//     }
// 	//若是有两个以上取投影面积最大的装甲版
//     else
//     {
//         Eigen::Vector2d uv[same_id_armors.size()];
//         double area[same_id_armors.size()];
//         for(int i = 0; i < same_id_armors.size(); i++)
//         {
//             for(int j = 0; j < 4; j++)
//             {
//                 uv[i][0] = same_id_armors[i].pts[j].x;
//                 uv[i][1] = same_id_armors[i].pts[j].y;
//             }
//             //对角线长
//             double main_diagonal = std::sqrt(std::pow(uv[0][0] - uv[2][0],2) + std::pow(uv[0][1] - uv[2][1],2));
//             double sub_diagonal = std::sqrt(std::pow(uv[1][0] - uv[1][0],2) + std::pow(uv[3][1] - uv[3][1],2));
//             double rd_ru = std::sqrt(std::pow(uv[2][0] - uv[3][0],2) + std::pow(uv[2][1] - uv[3][1],2));
//             double cos_theata = (std::pow((main_diagonal/2.0),2) + std::pow((sub_diagonal/2.0),2) - std::pow(rd_ru,2)) / 2*(main_diagonal/2.0)*(sub_diagonal/2.0);
//             double sin_theata = std::sqrt(1-std::pow(cos_theata,2));
//             area[i] = 0.5*main_diagonal*sub_diagonal*sin_theata;
//         }
//         unsigned int num = 0;
//         for (unsigned int i = 1; i < same_id_armors.size(); i++)
//         {
//             if (area[i] > area[num])
//             {
//                 num = i;
//             }
//         }
//         return objects[num];
//     } 
// }

// bool PredictorPose::isGyro(std::deque<double> time_buff_, Vector3d up_switch, Vector3d down_switch) //by lqq 未使用
// {
// 	{
// 		std::cout << GREENCOLOR << "4 --- 3: "<< std::abs(time_buff_[4] - time_buff_[3]) << std::endl;
// 		std::cout << GREENCOLOR << "3 --- 2: "<< std::abs(time_buff_[3] - time_buff_[2]) << std::endl;
// 		std::cout << GREENCOLOR << "2 --- 1: "<< std::abs(time_buff_[2] - time_buff_[1]) << std::endl;
// 		std::cout << GREENCOLOR << "1 --- 0: "<< std::abs(time_buff_[1] - time_buff_[0]) << std::endl;
// 		std::cout << WHITECOLOR << std::endl;
// 	}
	
// 	if ((std::abs(time_buff_[4] - time_buff_[3]) < 1.8 && std::abs(time_buff_[4] - time_buff_[3]) > 0.1)
// 		&& (std::abs(time_buff_[3] - time_buff_[2]) < 1.8 && std::abs(time_buff_[3] - time_buff_[2]) > 0.1) 
// 		&& (std::abs(time_buff_[2] - time_buff_[1]) < 1.8 && std::abs(time_buff_[2] - time_buff_[1]) > 0.1) 
// 		&& (std::abs(time_buff_[1] - time_buff_[0]) < 1.8 && std::abs(time_buff_[1] - time_buff_[0]) > 0.1))
// 	{
// 		return true;
// 	}
// 	else
// 	{
// 		return false;
// 	}
// }
// //在考虑空气阻力的情况下弹道结算，适用于英雄击打前哨战 by wty 未测试
// float PredictorPose::ballistic_equation(Eigen::Vector3d coord)
// {
// 	float T_fly;
// 	cv::Point3f p1;

// 	p1.x = coord[0];
// 	p1.y = coord[1];
// 	p1.z = coord[2];
// 	//先计算yaw轴角度 弧度制
// 	if(p1.z == 0)
// 	{
// 		p1.z += 0.000001;
// 	}
// 	gm_ptz.yaw = std::atan(-p1.x / p1.z);
// 	float distance1;
// 	distance1 = std::sqrt(p1.x * p1.x + p1.z * p1.z);
// 	//计算pitch初始角度
// 	gm_ptz.pitch = atan(p1.z / distance1);

// 	//考虑空气阻力计算角度
// 	double theta = gm_ptz.pitch;
// 	double delta_z;
// 	float v0_ = v0 - 0.5;

// 	//英雄弹丸规格 R = 42.5mm m = 41g
// 	//首先计算空气阻力系数
// 	double k1 = 0.47 * 1.169 * (2 * 3.14159f * 0.02125 * 0.02125) / 2 / 0.041;
// 	//使用的迭代法求解pitch轴
// 	for(int i = 0; i < 100; i++)
// 	{
// 		//计算飞行时间
// 		T_fly = (pow(2.718281828, k1 * distance1) - 1) / (k1 * v0_ * cos(theta));
// 		delta_z = p1.z - v0_ * sin(theta) * T_fly / cos(theta) + 4.9 * T_fly * T_fly / cos(theta) / cos(theta);

// 		//不断更新theta，直到delta_z小于某一个阈值
// 		if(fabs(delta_z) < 0.000001)
// 		{
// 			break;
// 		}
// 		theta -= delta_z / (-(v0_ * T_fly) / pow(cos(theta),2) + 9.8 * T_fly * T_fly / (v0_ * v0_) * sin(theta) / pow(cos(theta) , 3));
// 	}
// 	//调整弹道后的数据
// 	gm_ptz.pitch = theta;
// 	//将弧度制转换成角度制
// 	gm_ptz.pitch = (gm_ptz.pitch) / CV_PI * 180;
// 	gm_ptz.yaw = (gm_ptz.yaw) / CV_PI *180;

// 	return distance1 / (cos(theta) * v0_);
// }

//-----------------------------------------NT ----------------------------------------
//与isGyro类似
// /**
//  * @brief  反陀螺判断，由时间队列中的时间差判断是否为反陀螺
//  * @param  time_buff_
//  * @return bool
//  * @author AlexLi
//  * @date 2024-03-08
//  */
// bool PredictorPose::anGyro(std::deque<double> time_buff_)
// {
//     if (time_buff_[6] - time_buff_[5] < 0.5 && time_buff_[5] - time_buff_[4] < 0.5 &&
//         time_buff_[4] - time_buff_[3] < 0.5 && time_buff_[3] - time_buff_[2] < 0.5 &&
//         time_buff_[2] - time_buff_[1] < 0.5 && time_buff_[1] - time_buff_[0] < 0.5)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }
//和anGyro一样，暂未使用
// /**
//  * @brief  对时间队列进行排序
//  * @param  time_buff_
//  * @return double
//  * @author AlexLi
//  * @date 2024-03-08
//  */
// double PredictorPose::get_time(std::deque<double> time_buff_)
// {
//     double time = 0.0;
//     for (double i : time_buff_)
//     {
//         if (i > time)
//         {
//             time = i;
//         }
//     }
//     return time;
// }

//----------------------------------------NEXT-----------------------------------------
#ifdef NEXT_Gyro

// +++ 新增辅助函数
float PredictorPose::calculateYawDiff(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2) {
    float dist1 = sqrt(pt1[0]*pt1[0] + pt1[2]*pt1[2]);
    float yaw1 = asin(pt1[0]/dist1) * 180/CV_PI;
    float dist2 = sqrt(pt2[0]*pt2[0] + pt2[2]*pt2[2]);
    float yaw2 = asin(pt2[0]/dist2) * 180/CV_PI;
    return abs(yaw1 - yaw2);
}

float PredictorPose::calculateDistance(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2) {
    return sqrt(pow(pt1[0]-pt2[0],2) + pow(pt1[1]-pt2[1],2) + pow(pt1[2]-pt2[2],2));
}
// ---

#endif