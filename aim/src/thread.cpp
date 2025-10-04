#include "../include/thread.h"
#include "serial.h"

#ifdef TRT
#include "TRTModule.h"
#endif

#include <thread>
#include "../../rune/include/PowerRune.h"

//mutex image_mutex_{}; // 数据上🔓

// 世界坐标系内坐标--->相机坐标系内坐标
inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d &pw, const Eigen::Matrix3d &R_CW)
{
    cv::FileStorage fs("../control/aim_config.yaml", cv::FileStorage::READ);
    float X_BIAS, Y_BIAS, Z_BIAS;
    fs["X_BIAS"] >> X_BIAS;
    fs["Y_BIAS"] >> Y_BIAS;
    fs["Z_BIAS"] >> Z_BIAS;
	Eigen::Vector3d pw_t;
	pw_t = R_CW * pw;
	pw_t[0] = pw_t[0] - X_BIAS;
	pw_t[1] = pw_t[1] - Y_BIAS;
	pw_t[2] = pw_t[2] - Z_BIAS;
    fs.release();
	pw_t[1] = -pw_t[1];

	return pw_t;
}

// 相机坐标系内坐标--->图像坐标系内像素坐标
inline Eigen::Vector3d pc_to_pu(const Eigen::Vector3d &pc, const Eigen::Matrix3d &F)
{
	return F * pc / pc(2, 0);
}

namespace GxCamera
{
	// 大恒相机滑动条调参
	int GX_exp_time = 10000;
	int GX_gain = 10;
	DaHengCamera *camera_ptr_ = nullptr;
	int GX_blance_r = 50; // rbg颜色通道
	int GX_blance_g = 32;
	int GX_blance_b = 44;

	int GX_gamma = 1;

	// DaHengCamera* camera_ptr_ = nullptr;

	void DaHengSetExpTime(int, void *)
	{
		camera_ptr_->SetExposureTime(GX_exp_time);
	}

	void DaHengSetGain(int, void *)
	{
		camera_ptr_->SetGain(3, GX_gain);
	}

}
namespace MidCamera
{
	int MV_exp_value = 8000;
	MVCamera *camera_ptr_ = nullptr;

	int r = 152;
	int g = 125;
	int b = 100;

	int GAIN = 100;

	void MVSetExpTime(int, void *)
	{
		camera_ptr_->SetExpose(MV_exp_value);
		
	}

	void MVSetGain_R(int, void *)
	{
		camera_ptr_->SetGain(r,g,b);
	}

	void MVSetGain_G(int, void *)
	{
		camera_ptr_->SetGain(r,g,b);
	}

	void MVSetGain_B(int, void *)
	{
		camera_ptr_->SetGain(r,g,b);
	}
	
	void MVSetGain(int, void *)
	{
		camera_ptr_->SetGain(GAIN,GAIN,GAIN);
	}

}
namespace GenericCamera
{
	GNCamera *camera_ptr_ = nullptr;
}
using namespace camera;
namespace HKcamera
{
	HikCamera *MVS_cap = nullptr;												// 创建一个相机对象
	const string camera_config_path = "../drivers/HikVision/config/camera_config.yaml"; // 相机配置文件路径
	const string intrinsic_para_path = "../drivers/Distortions/camera_HK.yaml";				// 相机内参文件路径											// 记录相机初始化时间戳
	bool debug_flag = true;														// 是否开启相机调参
}

//[[noreturn]] void Factory::producer() //by lsn 全向通信出问题时可以尝试启用
void Factory::producer()
{
#ifdef DAHENG1
#ifdef Save_Record
    auto generateUniqueFilename = [](const std::string &prefix, int sequence)
    {
        std::stringstream ss;
        ss << prefix << "_" << sequence << ".avi";
        return ss.str();
    };
    int sequence = 1; // 初始序号
    // 读取保存序号的文件
    std::ifstream sequenceFile("../record/sequence.txt");
    if (sequenceFile.is_open())
    {
        sequenceFile >> sequence;
        sequenceFile.close();
    }
    int width = 1280;
    int height = 1024;
    std::string path = generateUniqueFilename("../record/output", sequence);
    int frame_cnt = 0;
    int squence_cnt = 0;
    auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 24.0, cv::Size(width, height)); // Avi format
    std::future<void> write_video;
    if (!writer.isOpened())
    {
        cerr << "Could not open the output video file for write\n";
        return;
    }
#endif
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	while (true)
	{
		if (GxCamera::camera_ptr_ != nullptr) // 打印图片
		{
			while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			{
			};
			if (GxCamera::camera_ptr_->GetMat(image_buffer_[image_buffer_front_ % IMGAE_BUFFER]))
			{
				// 调整后，把这段注释掉
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0);
				timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
				++image_buffer_front_;
				// 收数开始
#ifdef Save_Record
                frame_cnt++;
                squence_cnt++;
                cv::Mat src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
                if (squence_cnt % 1000 == 0) // 隔一段时间修改内陆所需要的顺序名
                {
                    sequence++;
                    std::ofstream sequenceFile("../record/sequence.txt");
                    if (sequenceFile.is_open())
                    {
                        sequenceFile << sequence;
                        sequenceFile.close();
                    }
                }
                if (frame_cnt % 10 == 0)
                {
                    frame_cnt = 0;
                    // 异步读写加速,避免阻塞生产者
                    writer.write(src);
                }

#endif
			}
			else
			{
				delete GxCamera::camera_ptr_;
				GxCamera::camera_ptr_ = nullptr;
			}
		}
		else
		{
			GxCamera::camera_ptr_ = new DaHengCamera;
			while (!GxCamera::camera_ptr_->StartDevice())
				;
			GxCamera::camera_ptr_->SetResolution();
			while (!GxCamera::camera_ptr_->StreamOn())
				;
			// 设置是否自动白平衡
			GxCamera::camera_ptr_->Set_BALANCE_AUTO(1);
			// 手动设置白平衡通道及系数，此之前需关闭自动白平衡

			GxCamera::camera_ptr_->SetExposureTime(GxCamera::GX_exp_time);
			GxCamera::camera_ptr_->SetGain(3, GxCamera::GX_gain);

			double GX_Gamma = 2.85;
			GxCamera::camera_ptr_->setGamma(GX_Gamma);

			cv::namedWindow("DaHengCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("DaHengExpTime", "DaHengCameraDebug", &GxCamera::GX_exp_time, 10000, GxCamera::DaHengSetExpTime);
			GxCamera::DaHengSetExpTime(0, nullptr);
			cv::createTrackbar("DaHengGain", "DaHengCameraDebug", &GxCamera::GX_gain, 10, GxCamera::DaHengSetGain);
			GxCamera::DaHengSetGain(0, nullptr);
			// GxCamera::DaHengSetGain(0,nullptr);

			image_buffer_front_ = 0;
			image_buffer_rear_ = 0;
		}
	}
#endif
#ifdef DAHENG2
#ifdef Save_Record
    auto generateUniqueFilename = [](const std::string &prefix, int sequence)
    {
        std::stringstream ss;
        ss << prefix << "_" << sequence << ".avi";
        return ss.str();
    };
    int sequence = 1; // 初始序号
    // 读取保存序号的文件
    std::ifstream sequenceFile("../record/sequence.txt");
    if (sequenceFile.is_open())
    {
        sequenceFile >> sequence;
        sequenceFile.close();
    }
    int width = 1280;
    int height = 1024;
    std::string path = generateUniqueFilename("../record/output", sequence);
    int frame_cnt = 0;
    int squence_cnt = 0;
    auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 24.0, cv::Size(width, height)); // Avi format
    std::future<void> write_video;
    if (!writer.isOpened())
    {
        cerr << "Could not open the output video file for write\n";
        return;
    }
#endif
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	while (true)
	{
		if (GxCamera::camera_ptr_ != nullptr) // 打印图片
		{
			while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			{
			};
			if (GxCamera::camera_ptr_->GetMat(image_buffer_[image_buffer_front_ % IMGAE_BUFFER]))
			{
				// 调整后，把这段注释掉
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0);
				timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
				++image_buffer_front_;
				// 收数开始
#ifdef Save_Record
                frame_cnt++;
                squence_cnt++;
                cv::Mat src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
                if (squence_cnt % 1000 == 0) // 隔一段时间修改内陆所需要的顺序名
                {
                    sequence++;
                    std::ofstream sequenceFile("../record/sequence.txt");
                    if (sequenceFile.is_open())
                    {
                        sequenceFile << sequence;
                        sequenceFile.close();
                    }
                }
                if (frame_cnt % 10 == 0)
                {
                    frame_cnt = 0;
                    // 异步读写加速,避免阻塞生产者
                    writer.write(src);
                }

#endif
			}
			else
			{
				delete GxCamera::camera_ptr_;
				GxCamera::camera_ptr_ = nullptr;
			}
		}
		else
		{
			GxCamera::camera_ptr_ = new DaHengCamera;
			while (!GxCamera::camera_ptr_->StartDevice())
				;
			GxCamera::camera_ptr_->SetResolution();
			while (!GxCamera::camera_ptr_->StreamOn())
				;
			// 设置是否自动白平衡
			GxCamera::camera_ptr_->Set_BALANCE_AUTO(1);
			// 手动设置白平衡通道及系数，此之前需关闭自动白平衡

			GxCamera::camera_ptr_->SetExposureTime(GxCamera::GX_exp_time);
			GxCamera::camera_ptr_->SetGain(3, GxCamera::GX_gain);

			double GX_Gamma = 2.85;
			GxCamera::camera_ptr_->setGamma(GX_Gamma);

			cv::namedWindow("DaHengCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("DaHengExpTime", "DaHengCameraDebug", &GxCamera::GX_exp_time, 10000, GxCamera::DaHengSetExpTime);
			GxCamera::DaHengSetExpTime(0, nullptr);
			cv::createTrackbar("DaHengGain", "DaHengCameraDebug", &GxCamera::GX_gain, 10, GxCamera::DaHengSetGain);
			GxCamera::DaHengSetGain(0, nullptr);
			// GxCamera::DaHengSetGain(0,nullptr);

			image_buffer_front_ = 0;
			image_buffer_rear_ = 0;
		}
	}
#endif

#ifdef MIDVISION

#ifdef Save_Record
    auto generateUniqueFilename = [](const std::string &prefix, int sequence)
    {
        std::stringstream ss;
        ss << prefix << "_" << sequence << ".avi";
        return ss.str();
    };
    int sequence = 1; // 初始序号
    // 读取保存序号的文件
    std::ifstream sequenceFile("../record/sequence.txt");
    if (sequenceFile.is_open())
    {
        sequenceFile >> sequence;
        sequenceFile.close();
    }
    int width = 1280;
    int height = 1024;
    std::string path = generateUniqueFilename("../record/output", sequence);
    int frame_cnt = 0;
    int squence_cnt = 0;
    auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 24.0, cv::Size(width, height)); // Avi format
    std::future<void> write_video;
    if (!writer.isOpened())
    {
        cerr << "Could not open the output video file for write\n";
        return;
    }
#endif

	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	while (true)
	{
		if (MidCamera::camera_ptr_ != nullptr)
		{

			// std::cout << "enter producer" << std::endl;
			while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER - 1)
			{
			};
			// bool is = image_buffer_.try_lock();
			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

			if (MidCamera::camera_ptr_->GetMat(image_buffer_[image_buffer_front_ % IMGAE_BUFFER]))
			{
				std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
				// std::cout << "time :" << time_run.count() << std::endl;

				MidCamera::camera_ptr_->SetExpose(MidCamera::MV_exp_value);

				// if (!is)
				// {
				// 	std::cout << "try lock failed!!" << std::endl;
				// }
				// std::cout << "enter producer lock" << std::endl;
				timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
				// cv::imshow("windowName", image_buffer_[image_buffer_front_ % IMGAE_BUFFER]);
				++image_buffer_front_;

				// std::cout << "out producer lock" << std::endl;
#ifdef Save_Record
                frame_cnt++;
                squence_cnt++;
                cv::Mat src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
                if (squence_cnt % 1000 == 0) // 隔一段时间修改内陆所需要的顺序名
                {
                    sequence++;
                    std::ofstream sequenceFile("../record/sequence.txt");
                    if (sequenceFile.is_open())
                    {
                        sequenceFile << sequence;
                        sequenceFile.close();
                    }
                }
                if (frame_cnt % 10 == 0)
                {
                    frame_cnt = 0;
                    // 异步读写加速,避免阻塞生产者
                    writer.write(src);
                }

#endif
			}
			else
			{
				delete MidCamera::camera_ptr_;
				MidCamera::camera_ptr_ = nullptr;
			}
			// image_mutex_.unlock();
		}
		else
		{
			MidCamera::camera_ptr_ = new MVCamera;

			MidCamera::camera_ptr_->SetExpose(5000);
#ifdef Img_Show
			cv::namedWindow("MVCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("MVExpTime", "MVCameraDebug", &MidCamera::MV_exp_value, 60000, MidCamera::MVSetExpTime);
#endif
			MidCamera::MVSetExpTime(0, nullptr);

#ifdef Img_Show
			cv::namedWindow("MVCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("r_gain", "MVCameraDebug", &MidCamera::r, 160, MidCamera::MVSetGain_R);

			cv::namedWindow("MVCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("g_gain", "MVCameraDebug", &MidCamera::g, 160, MidCamera::MVSetGain_G);
			
			cv::namedWindow("MVCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("b_gain", "MVCameraDebug", &MidCamera::b, 160, MidCamera::MVSetGain_B);
#endif

			image_buffer_front_ = 0;
			image_buffer_rear_ = 0;
		}
	}
#endif
#ifdef HK
#ifdef Save_Record
    auto generateUniqueFilename = [](const std::string &prefix, int sequence)
    {
        std::stringstream ss;
        ss << prefix << "_" << sequence << ".avi";
        return ss.str();
    };
    int sequence = 1; // 初始序号
    // 读取保存序号的文件
    std::ifstream sequenceFile("../record/sequence.txt");
    if (sequenceFile.is_open())
    {
        sequenceFile >> sequence;
        sequenceFile.close();
    }
    int width = 1280;
    int height = 1024;
    std::string path = generateUniqueFilename("../record/output", sequence);
    int frame_cnt = 0;
    int squence_cnt = 0;
    auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 24.0, cv::Size(width, height)); // Avi format
    std::future<void> write_video;
    if (!writer.isOpened())
    {
        cerr << "Could not open the output video file for write\n";
        return;
    }
#endif
	auto t0 = std::chrono::steady_clock::now(); // 记录相机初始化时间戳
	while (true)
	{
		if (HKcamera::MVS_cap != nullptr)
		{
			
			while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			{
				// std::cout << image_buffer_front_ - image_buffer_rear_ << std::endl;
			};
			if (HKcamera::MVS_cap->ReadImg(image_buffer_[image_buffer_front_ % IMGAE_BUFFER])) // 相机取图
			{
				auto t2 = std::chrono::steady_clock::now();
				std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0);
				// HKcamera::MVS_cap->undistProcess(image); // 相机畸变矫正示例(取消注释即可使用)
				timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
				++image_buffer_front_;
				
			}
			else
			{
				delete HKcamera::MVS_cap;
				HKcamera::MVS_cap = nullptr;
			}

#ifdef Save_Record
                frame_cnt++;
                squence_cnt++;
                cv::Mat src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
                if (squence_cnt % 1000 == 0) // 隔一段时间修改内陆所需要的顺序名
                {
                    sequence++;
                    std::ofstream sequenceFile("../record/sequence.txt");
                    if (sequenceFile.is_open())
                    {
                        sequenceFile << sequence;
                        sequenceFile.close();
                    }
                }
                if (frame_cnt % 10 == 0)
                {
                    frame_cnt = 0;
                    // 异步读写加速,避免阻塞生产者
                    writer.write(src);
                }

#endif
		}
		else
		{
			HKcamera::MVS_cap = new HikCamera;
			HKcamera::MVS_cap->Init(HKcamera::debug_flag, HKcamera::camera_config_path, HKcamera::intrinsic_para_path, t0); // 初始化相机，第一个参数为 动态调节相机参数模式
			HKcamera::MVS_cap->CamInfoShow();																				// 显示图像参数信息
			image_buffer_front_ = 0;
			image_buffer_rear_ = 0;
			
		}
	}
#endif 
#ifdef GENERIC
std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
while(true)
{

	while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER - 1)
	{
	};
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	if (GenericCamera::camera_ptr_->GetMat(image_buffer_[image_buffer_front_ % IMGAE_BUFFER]))
	{
		std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
		timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
		++image_buffer_front_;
	}
}
#endif

#ifdef VIDEO
	cv::VideoCapture cap("/home/ad/example5.mp4");
	cv::Mat src;
#ifdef SAVE_VIDEO
	// cv::Mat image;
	//    GxCamera::camera_ptr_->GetMat(image);
	cap >> src;
	std::cout << src.size().width << "   " << src.size().height << std::endl;
	int frame_cnt = 0;
	const std::string &storage_location = "../record/";
	char now[64];
	std::time_t tt;
	struct tm *ttime;
	int width = 1280;
	int height = 1024;
	tt = time(nullptr);
	ttime = localtime(&tt);
	strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime); // 以时间为名字
	std::string now_string(now);
	std::string path(std::string(storage_location + now_string).append(".avi"));
	auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(1350, 1080)); // Avi format
	std::future<void> write_video;
	if (!writer.isOpened())
	{
		cerr << "Could not open the output video file for write\n";
		return;
	}
#endif
	if (!cap.isOpened())
	{
		return;
	}
	for (;;)
	{
		while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			;
		cap >> image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
		src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
#ifdef SAVE_VIDEO
		frame_cnt++;
		if (frame_cnt % 10 == 0)
		{
			frame_cnt = 0;
			// 异步读写加速,避免阻塞生产者
			write_video = std::async(std::launch::async, [&, src]()
									 { writer.write(src); });
		}
#endif
		if (src.empty())
			break;

		image_buffer_front_++;
	}

#endif
}

void Factory::consumer()
{
	bool rune_window_is_have = false;
	#ifdef Rune
	power_rune::PowerRune pr;
	#endif
	
	#ifdef TRT
	TRTInferV1::TRTInfer myInfer(0);
	myInfer.initModule(EnginePath, 1, 4, 9, 4, 128);
	#endif

#ifdef GENERIC
    //cout << "Built with OpenCV " << CV_VERSION << endl;
    VideoCapture capture;
    capture.open(0);
    if(capture.isOpened())
    {
        cout << "Capture 0 is opened" << endl;
        //capture >> img;
    }
    else
    {
		capture.open(1);
		if(capture.isOpened())
		{
			cout << "Capture 1 is opened" << endl;
			//capture >> img;
		}
		else
		{
			cout << "No capture" << endl;
		}
    }

#endif
//int rune_shift_test_cnt = 1000;
while (true)
	{
		// 若满足这个条件，则让这个函数一只停在这里
		//std::cout << "enter consum lock" << std::endl;
		//image_mutex_.lock();
		while (image_buffer_front_ <= image_buffer_rear_)
		{
		};
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

		// 读取最新的图片
		image_buffer_rear_ = image_buffer_front_ - 1;
#ifndef GENERIC
#ifndef Nova_Get_Img
		// 直接获取引用（lqq，使用该方法时thread.h里不能有cv::Mat img;否则海康相机会出现无法画线的问题）
		cv::Mat &img = image_buffer_[image_buffer_rear_ % IMGAE_BUFFER];
#endif
#ifdef Nova_Get_Img
		cv::Mat img = image_buffer_[image_buffer_rear_ % IMGAE_BUFFER]; //mlx
#endif
#endif
#ifdef GENERIC
    	//cout << "Built with OpenCV " << CV_VERSION << endl;
		capture >> img;
#endif

		double src_time = timer_buffer_[image_buffer_rear_ % IMGAE_BUFFER];
		std::cout << "time :" << src_time << std::endl;
		serial_mutex_.lock();
		#ifdef Time_Sync_Serial
        stm32data = TimeSynchronization(MCU_data_, src_time);
		TimeSynchronization(MCU_data_, src_time); //lqq
		#endif
		#ifdef NT
		//stm32data = TimeSynchronization(MCU_data_, src_time);
		#endif

	#ifndef Inter_Vis_Ctl
		//imu_data为GimbalPose类，对象声明在thread.h
        imu_data.yaw = stm32data.yaw_data_.f;
        imu_data.pitch = stm32data.pitch_data_.f;
		imu_data.timestamp = stm32data.time.f;
		
		#ifdef NT
        myInfer.State_outpost = outpost_state;
		#endif

		#ifndef NT
        imu_data1.yaw = 0;
        imu_data1.pitch = 0;
		imu_data1.timestamp = stm32data.time.f;
		#endif

		if(stm32data.init_firing_rate>0)
		{
		predic_pose_->v0 = stm32data.init_firing_rate;
		#ifndef NT
		predic_pose_1->v0 = stm32data.init_firing_rate;
		predic_pose_2->v0 = stm32data.init_firing_rate;
		predic_pose_3->v0 = stm32data.init_firing_rate;
		#endif
		default_fire_v0 = stm32data.init_firing_rate;
		}
		else
		{
		predic_pose_->v0 = default_fire_v0;
		#ifndef NT
		predic_pose_1->v0 = default_fire_v0;
		predic_pose_2->v0 = default_fire_v0;
		predic_pose_3->v0 = default_fire_v0;
		#endif
		}
		cout<< CYAN<<"Bullet V: "<<predic_pose_->v0<<endl;
		std::cout << WHITECOLOR << std::endl;	
	#endif

	#ifdef Inter_Vis_Ctl
	    stm32data = TimeSynchronization(MCU_data_, src_time);
        imu_data.yaw = stm32data.YAW_DATA;
        imu_data.pitch = stm32data.PITCH_DATA;
		#ifdef NT
        myInfer.State_outpost = outpost_state;
		#endif
		#ifndef NT
        imu_data1.yaw = 0;
        imu_data1.pitch = 0;
		#endif
	#endif
//if(rune_shift_test_cnt>0)  //打符模式
if(stm32data.is_rune==1)  //打符模式
	{
		rune_window_is_have = true;
		//rune_shift_test_cnt--;
		serial_mutex_.unlock();
#ifdef Rune
		visiondata.is_have_armor=pr.runOnce(img, imu_data.pitch, imu_data.yaw);
		#ifndef Inter_Vis_Ctl
        visiondata.pitch_data_.f=pr.m_calculator.getPredictPitch();
        visiondata.yaw_data_.f=pr.m_calculator.getPredictYaw();
		#endif
		#ifdef Inter_Vis_Ctl
        visiondata.pitch_data_=pr.m_calculator.getPredictPitch();
        visiondata.yaw_data_=pr.m_calculator.getPredictYaw();
		#endif
		#ifdef Auto_Fire_Rune
		#ifndef Inter_Vis_Ctl
		if(visiondata.is_have_armor&std::abs(visiondata.yaw_data_.f-stm32data.yaw_data_.f)<fire_yaw_dif&&std::abs(visiondata.pitch_data_.f-stm32data.pitch_data_.f)<fire_pit_dif)
		{
			visiondata.is_fire = true;
		}
		else
		{
			visiondata.is_fire = false;
		}
		#endif
		#ifdef Inter_Vis_Ctl
		if(visiondata.is_have_armor&std::abs(visiondata.yaw_data_-stm32data.YAW_DATA)<fire_yaw_dif&&std::abs(visiondata.pitch_data_-stm32data.PITCH_DATA)<fire_pit_dif)
		{
			visiondata.is_fire = true;
		}
		else
		{
			visiondata.is_fire = false;
		}
		#endif
		#endif
#ifndef Inter_Vis_Ctl
		serial_mutex_.lock();
		data_controler_.sentData(fd, visiondata);
		serial_mutex_.unlock();
#endif
char test[100];
			#ifndef Inter_Vis_Ctl
			sprintf(test, "get yaw:%0.4f ", stm32data.yaw_data_.f);
			cv::putText(img, test, cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "get pitch:%0.4f ", stm32data.pitch_data_.f);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "send yaw:%0.4f ", visiondata.yaw_data_.f);
			cv::putText(img, test, cv::Point(10, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "send pitch:%0.4f ", visiondata.pitch_data_.f);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#endif
			#ifdef Inter_Vis_Ctl
			sprintf(test, "get yaw:%0.4f ", stm32data.YAW_DATA);
			cv::putText(img, test, cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "get pitch:%0.4f ", stm32data.PITCH_DATA);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "send yaw:%0.4f ", visiondata.yaw_data_);
			cv::putText(img, test, cv::Point(10, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "send pitch:%0.4f ", visiondata.pitch_data_);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#endif
			if (visiondata.is_have_armor)
			{
				sprintf(test, "existence:%s ", "true");
			}
			else
			{
				sprintf(test, "existence:%s ", "false");
			}
			cv::putText(img, test, cv::Point(3*img.cols / 4, 480), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			if (visiondata.is_fire == 0)
			{
				sprintf(test, " is fire:%s ", "false");
			}
			else
			{
				sprintf(test, " is fire:%s ", "true");
			}
			cv::putText(img, test, cv::Point(10, 440), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#ifndef Inter_Vis_Ctl
			if (stm32data.is_get)
			{
				sprintf(test, " is get:%s ", "true");
			}
			else
			{
				sprintf(test, " is get:%s ", "false");
			}
			cv::putText(img, test, cv::Point(10, 400), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#endif
			if (stm32data.is_rune)
			{
				sprintf(test, " is rune:%s ", "true");
			}
			else
			{
				sprintf(test, " is rune:%s ", "false");
			}
			cv::putText(img, test, cv::Point(3*img.cols / 4, 600), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
#endif
	}
else //自瞄模式
	{
		#ifdef TRT
		if (detect_color == 1)
		{
			myInfer.color_id = COLOR::RED_TRT;
		}
		else if (detect_color == 2)
		{
			myInfer.color_id = COLOR::BLUE_TRT;
		}
		else if (detect_color == 3)
		{
			myInfer.color_id = COLOR::PURPLE_TRT;
		}
		else
		{
			myInfer.color_id = COLOR::GRAY_TRT;
		}
		#endif
		
		#ifdef OV
		if (detect_color == 1)
		{
			infer.set_color_ = COLOR::RED_OV;
		}
		else if (detect_color == 2)
		{
			infer.set_color_ = COLOR::BLUE_OV;
		}
		else
		{
			infer.set_color_ = COLOR::GRAY_OV;
		}
		#endif

		serial_mutex_.unlock();

		//std::cout << "stm32 speed is: " << stm32data.init_firing_rate << std::endl;
		
		// 神经网络读取图像
		frames.clear(); 
		frames.emplace_back(img); 

		#ifdef TRT
		auto armors = myInfer.doInference(frames, TRT_confidence, TRT_nms_threshold);
		#endif

		#ifdef OV
		std::vector<DetectObject> armors = infer.run(img);
		//gim_cur = predic_pose_->run_current(imu_data, armors, src_time); //处理接收数据后装甲板中心数据
		gim_pre = predic_pose_->run_predict(imu_data, armors, src_time); //处理接收数据后装甲板预测数据
		gim_cur = predic_pose_1->run_current(imu_data, armors, src_time); //处理接收数据后装甲板中心数据
		gim_raw_cur = predic_pose_2->run_current(imu_data1, armors, src_time); //原始的装甲板中心数据
		//gim_raw_pre = predic_pose_3->run_predict(imu_data1, armors, src_time); //原始的装甲板预测数据

		// coord = predic_pose_->last_pose_.first;//old
		// rotation = predic_pose_->last_pose_.second;//old
		#endif
		#ifndef NT

		#ifdef TRT
		if (armors.size() == 1)
		{
			//gim_cur = predic_pose_->run_current(imu_data, armors[0], src_time); //处理接收数据后装甲板中心数据
			gim_pre = predic_pose_->run_predict(imu_data, armors[0], src_time); //处理接收数据后装甲板预测数据
			gim_cur = predic_pose_1->run_current(imu_data, armors[0], src_time); //处理接收数据后装甲板中心数据
			gim_raw_cur = predic_pose_2->run_current(imu_data1, armors[0], src_time); //原始的装甲板中心数据
			//gim_raw_pre = predic_pose_3->run_predict(imu_data1, armors[0], src_time); //原始的装甲板预测数据

			// coord = predic_pose_->last_pose_.first;//old
			// rotation = predic_pose_->last_pose_.second;//old
			
		}
		#endif

		#ifdef TRT
		if (armors[0].size() == 0)
		{
			visiondata.is_have_armor = false;
		}
		#endif
		#ifdef OV
		if (armors.size() == 0)
		{
			visiondata.is_have_armor = false;
		}
		#endif
		else
		{

				serial_mutex_.lock(); //nova
				#ifdef Pit_Lock
				if (std::abs(gim_cur.pitch) > 20) //锁定p轴上下限，仅用于全向步兵
				{
					gim_cur.pitch = 20; 
				}
				#endif
#ifdef Pure_Predict_Mode
			#ifndef Inter_Vis_Ctl
			visiondata.yaw_data_.f = gim_pre.yaw;
			visiondata.pitch_data_.f = gim_pre.pitch;
			#endif
			#ifdef Inter_Vis_Ctl
			visiondata.yaw_data_ = gim_pre.yaw;
			visiondata.pitch_data_ = gim_pre.pitch;
			#endif
#endif
#ifdef Pure_Track_Mode
			#ifndef Inter_Vis_Ctl
			visiondata.yaw_data_.f = gim_cur.yaw;
			visiondata.pitch_data_.f = -gim_cur.pitch;
			#endif
			#ifdef Inter_Vis_Ctl
			visiondata.yaw_data_ = gim_cur.yaw;
			visiondata.pitch_data_ = gim_cur.pitch;
			#endif
#endif

#ifdef Edge_Track_Mode
			#ifndef Inter_Vis_Ctl
			if(std::abs(gim_raw_cur.yaw)<yaw_max_edge&std::abs(gim_cur.yaw-gim_pre.yaw)<pre_yaw_maxdif)
			{
				visiondata.yaw_data_.f = gim_pre.yaw;
				std::cout << BLUECOLOR << "Send Prediction" << std::endl;
				std::cout << WHITECOLOR << std::endl;
			}
			else
			{
				visiondata.yaw_data_.f = gim_cur.yaw;
				std::cout << YELLOWCOLOR << "Send Current" << std::endl;
				std::cout << WHITECOLOR << std::endl;
			}

			visiondata.pitch_data_.f = gim_pre.pitch;
			// if(std::abs(gim_raw_cur.yaw)<yaw_max_edge&std::abs(gim_cur.yaw-gim_pre.yaw)<pre_yaw_maxdif)
			// {
			// 	visiondata.yaw_data_.f = gim_pre.yaw;
			// }
			// else
			// {
			// 	visiondata.yaw_data_.f = gim_cur.yaw;
			// }
			#endif
			#ifdef Inter_Vis_Ctl
			if(std::abs(gim_raw_cur.yaw)<yaw_max_edge&std::abs(gim_cur.yaw-gim_cur.yaw)<pre_yaw_maxdif)
			{
				visiondata.yaw_data_ = gim_pre.yaw;
			}
			else
			{
				visiondata.yaw_data_ = gim_cur.yaw;
			}

			visiondata.pitch_data_ = gim_pre.pitch;
			// if(std::abs(gim2.yaw)<yaw_max_edge&std::abs(gim.yaw-gim1.yaw)<pre_yaw_maxdif)
			// {
			// 	visiondata.yaw_data_.f = gim1.yaw;
			// }
			// else
			// {
			// 	visiondata.yaw_data_.f = gim.yaw;
			// }
			#endif
#endif
			#ifndef Inter_Vis_Ctl
			visiondata.time.f = src_time;
			#endif
			visiondata.is_have_armor = true;

			#ifdef Auto_Fire_Normal
			#ifndef Inter_Vis_Ctl
			if(visiondata.is_have_armor&std::abs(visiondata.yaw_data_.f-stm32data.yaw_data_.f)<fire_yaw_dif&&std::abs(visiondata.pitch_data_.f-stm32data.pitch_data_.f)<fire_pit_dif)
			{
				visiondata.is_fire = true;
			}
			else
			{
				visiondata.is_fire = false;
			}
			#endif
			#ifdef Inter_Vis_Ctl
			if(visiondata.is_have_armor&std::abs(visiondata.yaw_data_-stm32data.YAW_DATA)<fire_yaw_dif&&std::abs(visiondata.pitch_data_-stm32data.PITCH_DATA)<fire_pit_dif)
			{
				visiondata.is_fire = true;
			}
			else
			{
				visiondata.is_fire = false;
			}
			#endif
			#endif
			#ifdef Auto_Fire_Gyro
				#ifdef Inter_Vis_Ctl
				visiondata.is_have_armor = predic_pose_->is_fire;
				#endif
				#ifndef Inter_Vis_Ctl
				visiondata.is_fire = predic_pose_->is_fire;
				#endif
			#endif

				serial_mutex_.unlock(); //nova
		}
		#ifndef Inter_Vis_Ctl
		serial_mutex_.lock();
		data_controler_.sentData(fd, visiondata);
		serial_mutex_.unlock();
		#endif
	#endif
	#ifdef NT

        // if (!armors[0].empty())
        // {
        //     predic_pose_->state = ARMOR_STATE_::TRACK;
        // }
        // if (predic_pose_->state != ARMOR_STATE_::LOSS)
        // {
        //     gim = predic_pose_->run(imu_data, armors[0], src_time);
        //     coord = predic_pose_->last_pose_.first;
        //     rotation = predic_pose_->last_pose_.second;
        //     visiondata.is_have_armor = true;
        // }
        // else
        // {
        //     visiondata.is_have_armor = false;
        // }
        gim = predic_pose_->run(imu_data, armors[0], src_time);
        if (predic_pose_->state == ARMOR_STATE_::LOSS)
        {
            visiondata.is_have_armor = false;
        }
        else
        {
            visiondata.is_have_armor = true;
            coord = predic_pose_->last_pose_.first;
            rotation = predic_pose_->last_pose_.second;
        }
		#ifdef Pit_Lock
        if (std::abs(gim.pitch) > 20)
        {
            gim.pitch = 20;
        }
		#endif
        serial_mutex_.lock();
		#ifndef Inter_Vis_Ctl
        visiondata.yaw_data_.f = gim.yaw;
        visiondata.pitch_data_.f = gim.pitch;
		visiondata.time.f = src_time;
		#endif
		#ifdef Inter_Vis_Ctl
        visiondata.yaw_data_ = gim.yaw;
        visiondata.pitch_data_ = gim.pitch;
		#endif

		#ifdef Auto_Fire_Normal
		#ifndef Inter_Vis_Ctl
		if(visiondata.is_have_armor&std::abs(visiondata.yaw_data_.f-stm32data.yaw_data_.f)<fire_yaw_dif&&std::abs(visiondata.pitch_data_.f-stm32data.pitch_data_.f)<fire_pit_dif)
		{
			visiondata.is_fire = true;
		}
		else
		{
			visiondata.is_fire = false;
		}
		#endif
		#ifdef Inter_Vis_Ctl
		if(visiondata.is_have_armor&std::abs(visiondata.yaw_data_-stm32data.YAW_DATA)<fire_yaw_dif&&std::abs(visiondata.pitch_data_-stm32data.PITCH_DATA)<fire_pit_dif)
		{
			visiondata.is_fire = true;
		}
		else
		{
			visiondata.is_fire = false;
		}
		#endif
		#endif
		// #ifdef Auto_Fire_Gyro
		// 	#ifdef Inter_Vis_Ctl
		// 	visiondata.is_have_armor = predic_pose_->is_fire;
		// 	#endif
		// 	#ifndef Inter_Vis_Ctl
		// 	visiondata.is_fire = predic_pose_->is_fire;
		// 	#endif
		// #endif
        serial_mutex_.unlock();

#ifndef Inter_Vis_Ctl
		serial_mutex_.lock();
		data_controler_.sentData(fd, visiondata);
		serial_mutex_.unlock();
#endif

	#endif


#ifdef UI_Show

		char test[100];
			#ifdef NT
			sprintf(test, "x:%0.4f", coord[0]);			
			cv::putText(img, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "y:%0.4f", coord[1]);
			cv::putText(img, test, cv::Point(2 * img.cols / 5, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "z:%0.4f", coord[2]);
			cv::putText(img, test, cv::Point(3 * img.cols / 4, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "roll:%0.4f", rotation[0] * 180 / CV_PI);
			cv::putText(img, test, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "pitch:%0.4f", rotation[1] * 180 / CV_PI);
			cv::putText(img, test, cv::Point(2 * img.cols / 5, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "yaw:%0.4f", rotation[2] * 180 / CV_PI);
			cv::putText(img, test, cv::Point(3 * img.cols / 4, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#endif
			#ifndef NT
			// sprintf(test, "x:%0.4f", coord[0]);			
			sprintf(test, "x:%0.4f", predic_pose_->last_pose_.first[0]);
			cv::putText(img, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			// sprintf(test, "y:%0.4f", coord[1]);
			sprintf(test, "y:%0.4f", predic_pose_->last_pose_.first[1]);
			cv::putText(img, test, cv::Point(2 * img.cols / 5, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			// sprintf(test, "z:%0.4f", coord[2]);
			sprintf(test, "z:%0.4f", predic_pose_->last_pose_.first[2]);
			cv::putText(img, test, cv::Point(3 * img.cols / 4, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			// sprintf(test, "roll:%0.4f", rotation[0] * 180 / CV_PI);
			sprintf(test, "roll:%0.4f", predic_pose_->last_pose_.second[0] * 180 / CV_PI);
			cv::putText(img, test, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			// sprintf(test, "pitch:%0.4f", rotation[1] * 180 / CV_PI);
			sprintf(test, "pitch:%0.4f", predic_pose_->last_pose_.second[1] * 180 / CV_PI);
			cv::putText(img, test, cv::Point(2 * img.cols / 5, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			// sprintf(test, "yaw:%0.4f", rotation[2] * 180 / CV_PI);
			sprintf(test, "yaw:%0.4f", predic_pose_->last_pose_.second[2] * 180 / CV_PI);
			cv::putText(img, test, cv::Point(3 * img.cols / 4, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#endif
			#ifndef Inter_Vis_Ctl
			sprintf(test, "get yaw:%0.4f ", stm32data.yaw_data_.f);
			cv::putText(img, test, cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "get pitch:%0.4f ", stm32data.pitch_data_.f);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "send yaw:%0.4f ", visiondata.yaw_data_.f);
			cv::putText(img, test, cv::Point(10, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "send pitch:%0.4f ", visiondata.pitch_data_.f);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#ifndef NT
			sprintf(test, "current yaw:%0.4f ", gim_cur.yaw);
			cv::putText(img, test, cv::Point(10, 280), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "predict yaw:%0.4f ", gim_pre.yaw);
			cv::putText(img, test, cv::Point(10, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			// sprintf(test, "orin current yaw:%0.4f ", gim_raw_cur.yaw);
			// cv::putText(img, test, cv::Point(10, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			// sprintf(test, "orin predict yaw:%0.4f ", gim_raw_pre.yaw);
			// cv::putText(img, test, cv::Point(2 * img.cols / 5, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#endif
			#endif
			#ifdef Inter_Vis_Ctl
			sprintf(test, "get yaw:%0.4f ", stm32data.YAW_DATA);
			cv::putText(img, test, cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "get pitch:%0.4f ", stm32data.PITCH_DATA);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "send yaw:%0.4f ", visiondata.yaw_data_);
			cv::putText(img, test, cv::Point(10, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "send pitch:%0.4f ", visiondata.pitch_data_);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#ifndef NT
			sprintf(test, "current yaw:%0.4f ", gim_cur.yaw);
			cv::putText(img, test, cv::Point(10, 280), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "predict yaw:%0.4f ", gim_pre.yaw);
			cv::putText(img, test, cv::Point(10, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			// sprintf(test, "orin current yaw:%0.4f ", gim_raw_cur.yaw);
			// cv::putText(img, test, cv::Point(10, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			// sprintf(test, "orin predict yaw:%0.4f ", gim_raw_pre.yaw);
			// cv::putText(img, test, cv::Point(2 * img.cols / 5, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#endif
			#endif

			sprintf(test, "x speed:%0.4f ", predic_pose_->last_velocity_[0] * 100);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#ifndef NT
			sprintf(test, "move:%0.4f ", predic_pose_->move_);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#endif
			
			if (predic_pose_->pnp_solve_->is_large_)
			{
				sprintf(test, "armor size:%s ", "large");
			}
			else
			{
				sprintf(test, "armor size:%s ", "small");
			}
			cv::putText(img, test, cv::Point(3*img.cols / 4, 440), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			//#ifndef Inter_Vis_Ctl
			if (visiondata.is_have_armor)
			{
				sprintf(test, "existence:%s ", "true");
			}
			else
			{
				sprintf(test, "existence:%s ", "false");
			}
			cv::putText(img, test, cv::Point(3*img.cols / 4, 480), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			//#endif
			// #ifdef Inter_Vis_Ctl
			// if (visiondata.is_have_armor)
			// {
			// 	sprintf(test, "fire:%s ", "true");
			// }
			// else
			// {
			// 	sprintf(test, "fire:%s ", "false");
			// }
			// cv::putText(img, test, cv::Point(3*img.cols / 4, 480), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			// #endif

			#ifndef Inter_Vis_Ctl
			if (stm32data.is_get)
			{
				sprintf(test, " is get:%s ", "true");
			}
			else
			{
				sprintf(test, " is get:%s ", "false");
			}
			cv::putText(img, test, cv::Point(10, 400), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			#endif
		#ifdef Gyro_UI	
			if (predic_pose_->is_gyro_)
			{
				sprintf(test, " is gyro:%s ", "true");
			}
			else
			{
				sprintf(test, " is gyro:%s ", "false");
			}
			cv::putText(img, test, cv::Point(3*img.cols / 4, 600), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

			sprintf(test, "time diff:%0.4f ", predic_pose_->time_diff);
			cv::putText(img, test, cv::Point(3*img.cols / 4, 560), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

		#endif
			//#ifndef Inter_Vis_Ctl
			if (visiondata.is_fire == 0)
			{
				sprintf(test, " is fire:%s ", "false");
			}
			else
			{
				sprintf(test, " is fire:%s ", "true");
			}
			cv::putText(img, test, cv::Point(10, 440), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
			//#endif

#endif
		//整车估计
		#ifdef Veh_Estim
		for (int i = 0; i < 4; i++)
		{
			Eigen::Vector3d pc = pw_to_pc(predic_pose_->car_four_armors_[i], predic_pose_->transform_vector_inv_);
			Eigen::Matrix3d F;
			cv2eigen(predic_pose_->pnp_solver_.K_, F);
			Eigen::Vector3d pu = pc_to_pu(pc, F);
			#ifdef UI_Show
			cv::circle(frames[0], {int(pu(0, 0)), int(pu(1, 0))}, 3, cv::Scalar(0, 165, 255), cv::LINE_AA);
			#endif
		}
		#endif

		//if (true) //显示无装甲板前瞬间的点
		if (visiondata.is_have_armor == 1) //无装甲板时不显示任何点
		{
				#ifndef NT
				// if (predic_pose_->switch_flag)
				// {
				// 	cv::circle(img, {int(30), int(800)}, 3, cv::Scalar(0, 165, 255), cv::LINE_AA); 
				// 	//装甲板切换触发与否的标志点，在屏幕右下出现时表面触发装甲板切换机制（即isSwitch函数），仅为标识，无实际作用
				// }
				if (predic_pose_->switch_flag == 1)
				{
					sprintf(test, "switch");
					cv::putText(img, test, cv::Point(30, 800), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 165, 200), 2, 8);
				}

				//绘点逻辑：将世界坐标系内一点，投影到图像中，并绘制该点
				//装甲板中心转完坐标系的点
				Eigen::Vector3d pc1 = pw_to_pc(predic_pose_1->current_location_, predic_pose_1->transform_vector_);
				Eigen::Matrix3d F1;
				cv2eigen(predic_pose_1->pnp_solve_->K_, F1); 
				Eigen::Vector3d pu1 = pc_to_pu(pc1, F1);
				#ifdef UI_Show
				cv::circle(img, {int(pu1(0, 0)), int(pu1(1, 0))}, 3, cv::Scalar(255, 255, 255), cv::LINE_AA);
				#endif
				#endif


				//预测点（红点）
				Eigen::Vector3d pc = pw_to_pc(predic_pose_->predict_location_, predic_pose_->transform_vector_);
				Eigen::Matrix3d F;
				cv2eigen(predic_pose_->pnp_solve_->K_, F);
				Eigen::Vector3d pu = pc_to_pu(pc, F);
				cv::circle(img, {int(pu(0, 0)), int(pu(1, 0))}, 3, cv::Scalar(0, 0, 255), cv::LINE_AA); 


				#ifdef Gyro_UI
				if (predic_pose_->is_gyro_)
				{
					#ifndef NEXT_Gyro
					cv::circle(img, {int(60), int(900)}, 6, cv::Scalar(128, 0, 128), cv::LINE_AA);

					Eigen::Vector3d pc1 = pw_to_pc(predic_pose_->left_switch_, predic_pose_->transform_vector_);
					Eigen::Matrix3d F1;
					cv2eigen(predic_pose_->pnp_solve_->K_, F1);
					Eigen::Vector3d pu1 = pc_to_pu(pc1, F1);
					cv::circle(img, {int(pu1(0, 0)), int(pu1(1, 0) - 70)}, 3, cv::Scalar(255, 0, 0), cv::LINE_AA);

					Eigen::Vector3d pc2 = pw_to_pc(predic_pose_->right_switch_, predic_pose_->transform_vector_);
					Eigen::Matrix3d F2;
					cv2eigen(predic_pose_->pnp_solve_->K_, F2);
					Eigen::Vector3d pu2 = pc_to_pu(pc2, F2);
					cv::circle(img, {int(pu2(0, 0)), int(pu2(1, 0) - 70)}, 3, cv::Scalar(255, 255, 255), cv::LINE_AA);
					#endif
					#ifdef NEXT_Gyro
					if (predic_pose_->is_gyro_ == 1)
					{
						sprintf(test, "Gyro");
						cv::putText(img, test, cv::Point(30, 840), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 100, 100), 2, 8);
					}
					//反陀螺模式下锁住的敌车中心点
					Eigen::Vector3d pc2 = pw_to_pc(predic_pose_->predict_location_, predic_pose_->transform_vector_);
					Eigen::Matrix3d F;
					cv2eigen(predic_pose_->pnp_solve_->K_, F);
					Eigen::Vector3d pu = pc_to_pu(pc2, F);
					cv::circle(img, {int(pu(0, 0)), int(pu(1, 0))}, 3, cv::Scalar(0, 0, 255), cv::LINE_AA); 
					#endif
				}
				else
				{
					// Eigen::Vector3d pc1 = pw_to_pc(predic_pose_->current_location_, predic_pose_->transform_vector_);
					// Eigen::Matrix3d F1;
					// cv2eigen(predic_pose_->pnp_solve_->K_, F1); 
					// Eigen::Vector3d pu1 = pc_to_pu(pc1, F1);
					// cv::circle(img, {int(pu1(0, 0)), int(pu1(1, 0))}, 3, cv::Scalar(255, 255, 255), cv::LINE_AA);
				}
				#endif
		}
		else
		{
        #ifdef Last_Pose
		//上一帧预测点
		Eigen::Vector3d pc = pw_to_pc(predic_pose_->last_pose_.first, predic_pose_->transform_vector_);
		Eigen::Matrix3d F;
		cv2eigen(predic_pose_->pnp_solve_->K_, F);
		Eigen::Vector3d pu = pc_to_pu(pc, F);
		#ifdef UI_Show
		cv::circle(img, {int(pu(0, 0)), int(pu(1, 0))}, 3, cv::Scalar(0, 255, 255), cv::LINE_AA);
		#endif
		#endif
		}

}
#ifdef Img_Show
	if(stm32data.is_rune==0)
	{
		if(rune_window_is_have == true)
		{
			destroyWindow("Rune Mode");
			rune_window_is_have = false;
		}
		
		std::string windowName = "Auto Aim Mode";
		cv::namedWindow(windowName, 0);
		cv::imshow(windowName, img);
	}
		
#endif
		#ifdef Img_Show
		cv::waitKey(1);
		#endif
		#ifdef S_Frame
		cv::waitKey(0);
		#endif
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

		float FPS = 1 / (time_run.count());

		std::cout << "                 "
				  << "FPS: " << FPS << std::endl;

	}
}
#ifndef Inter_Vis_Ctl
void Factory::sr_serial()
{

	//循环嵌套自适应串口
	fd = OpenPort("/dev/ttyACM0");
	if (fd == -1)
	{
		fd = OpenPort("/dev/ttyACM1");
		if (fd == -1)
		{
			fd = OpenPort("/dev/ttyUSB0");
			if (fd == -1)
			{
				fd = OpenPort("/dev/ttyUSB1");
				if (fd == -1)
				{
					fd = OpenPort("/dev/ttyTHS0");
					if (fd == -1)
					{
						fd = OpenPort("/dev/ttyTHS1");
					}
				}
			}
		}
	}

	
	configureSerial(fd);
	while (1)
	{
		// cv::waitKey(1);
		// cv::waitKey(2);
		if (fd == -1)
		{
			continue;
		}

		serial_mutex_.lock();
		data_controler_.getData(fd, stm32data);
		#ifdef Time_Sync_Serial
		if (!stm32data_temp.is_aim)
		{
		// stm32data_temp.yaw_data_.f = 0;
		// stm32data_temp.pitch_data_.f = 0;
		is_aim_ = false;
		// imu_data.yaw = stm32data_temp.yaw_data_.f;
		// imu_data.pitch = stm32data_temp.pitch_data_.f;
		}
		else
		{
			is_aim_ = true;
		}

		if (!stm32data_temp.is_get)
		{
			// std::cout << "is_not_receive" << std::endl;
			stm32data = last_stm32_;
		 	serial_mutex_.unlock();
		 	continue;
		}
		else
		{
			last_stm32_ = stm32data;
			// std::cout << "is_received" << std::endl;
		}

		if (MCU_data_.size() < mcu_size_)
		{
			MCU_data_.push_back(stm32data_temp);
		}
		else
		{
			MCU_data_.pop_front();
		 	MCU_data_.push_back(stm32data_temp);
		}
		#endif

		serial_mutex_.unlock();
	}
}
#endif
#ifdef Inter_Vis_Ctl
[[noreturn]] void Factory::sr_IVC()
{
    int sockfd;
    // 创建Socket并初始化
    struct sockaddr_in servaddr
    {
    };
    while (true)
    {
        if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
        {
            perror("socket");
        }
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(54687);
        servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1) // 尝试连接到服务器
        {
            perror("connect");
            close(sockfd);
            sleep(1); // 等待一段时间后尝试重新连接
            continue; // 继续循环尝试重连
        }
        while (true)
        {
            Send_Receive(sockfd);
            usleep(10000);
        }
    }
}
Stm32Data Factory::TimeSynchronization(std::deque<Stm32Data> &stm32s, double src_time)
{

    if ((int)stm32s.size() == 0)
    {
        Stm32Data init{};
        return init;
    }
    else
    {
        int index = 0;

        vector<double> scale_time;
        scale_time.reserve(1000);

        for (int i = 0; i < (int)stm32s.size(); i++)
        {
            scale_time[i] = src_time - stm32s[i].time;
        }

        for (int i = 0; i < (int)stm32s.size(); i++)
        {
            if (std::abs(scale_time[i]) < std::abs(scale_time[index]))
            {
                index = i;
            }
        }

        Stm32Data stm32 = stm32s[index];
        stm32.PITCH_DATA = stm32s[index].PITCH_DATA;
        stm32.YAW_DATA = stm32s[index].YAW_DATA;
        stm32.time = stm32s[index].time;
        return stm32;
    }
}
void Factory::DebugVofa()
{
	FuncSendToVofa();
}
#endif

#ifdef Time_Sync_Serial
Horizon::DataControler::Stm32Data Factory::TimeSynchronization(std::deque<Horizon::DataControler::Stm32Data> &stm32s, double src_time)
{
    //std::cout << "stm32s size() " << stm32s.size() << std::endl;
	if (stm32s.size() == 0)
	{
		Horizon::DataControler::Stm32Data a;
		return a;
	}
	int index = 0;

	// for(auto stm32 : stm32s)
	// {
	// 	if(!stm32.is_get)
	// 	{
	// 		// std::cout << "true" << std::endl;
	// 	}
	// }

	vector<double> scale_time;
	scale_time.reserve(1000);

	for (int i = 0; i < stm32s.size(); i++)
	{
		scale_time[i] = src_time - stm32s[i].time.f;
	}

	for (int i = 0; i < stm32s.size(); i++)
	{
		if (std::abs(scale_time[i]) < std::abs(scale_time[index]))
		{
			index = i;
		}
	}
	std::cout << "finished!!" << std::endl;
	Horizon::DataControler::Stm32Data stm32 = stm32s[index];

	// stm32data.is_get = stm32s[index].is_get;
	// stm32data.pitch_data_.f = stm32s[index].pitch_data_.f;
	// stm32data.yaw_data_.f = stm32s[index].yaw_data_.f;
	// stm32data.time.f = stm32s[index].time.f;
	// stm32data.init_firing_rate = stm32s[index].init_firing_rate;

	stm32data = stm32;

	return stm32;
}
#endif
