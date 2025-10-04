#include "serial.h"

#ifdef TRT
#include "../trt/TRTModule.h"
#endif

#ifndef Inter_Vis_Ctl
namespace Horizon
{
#define DATA_LENGTH 16 // 接收的数据位数
#define SERIAL_RECIVER_TRANSFER_TIME 0.001875f

	int OpenPort(const char *Portname)
	{
		int fd;
		//    fd = open(Portname,O_RDWR);
		fd = open(Portname, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (-1 == fd)
		{
			printf("串口打开失败\n");
			return -1;
		}
		else
		{
			printf("串口打开成功\n");
			cout << "串口序号："<<fd << endl;
			fcntl(fd, F_SETFL, FNDELAY); // 读取串口的信息
		}
		return fd;
	}

	int configureSerial(int fd)
	{
		struct termios port_settings;		  // 用于存储端口设置的结构体
		//波特率
		cfsetispeed(&port_settings, B115200); 
		cfsetospeed(&port_settings, B115200);
		/* 启用接收器并设置本地模式...*/

		port_settings.c_cflag |= (CLOCAL | CREAD);
		/* Set c_cflag options.*/
		port_settings.c_cflag &= ~PARENB; // set no parity, stop bits, data bits  //无奇偶校验
		port_settings.c_cflag &= ~PARODD;
		port_settings.c_cflag &= ~CSTOPB; //停止位:1bit
		port_settings.c_cflag &= ~CSIZE; //清除数据位掩码
		port_settings.c_cflag |= CS8;
		// port_settings.c_cflag &= ~CRTSCTS;

		//port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
		/* open soft flow control */
		port_settings.c_iflag |= (IXON | IXOFF | IXANY);

		port_settings.c_iflag &= ~(INLCR | IGNCR | ICRNL);
		port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		port_settings.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP);
		/* Set c_oflag output options */
		port_settings.c_oflag &= ~OPOST;
		/* Set the timeout options */
		// port_settings.c_cc[VTIME] = 0;
		// port_settings.c_cc[VMIN] = 0;

		/* flow start with 0x11, end with 0x13 */
		port_settings.c_cc[VSTART] = 0x11;
		port_settings.c_cc[VSTOP] = 0x13;

		tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

		tcflush(fd, TCIFLUSH);

		return (fd);
	}

	void DataControler::sentData(int fd, VisionData data)
	{

		unsigned char send_bytes[15] = {0};

		if (false)
		{
			send_bytes[0] = 0xcd;
			send_bytes[14] = 0xdc;
		}
		else
		{
			send_bytes[0] = 0xcd;
			send_bytes[14] = 0xdc;
		}
		#ifdef Plane
        uint8_t tempBuffer[4]; // 用于存储单个浮点数的4个字节

		// 将 centeredIntersection.x 转换为字节序列并复制到 send_bytes[1] 到 send_bytes[4]
	    // memcpy(tempBuffer, &centeredIntersection.x, sizeof(float));
	    // std::copy(tempBuffer, tempBuffer + sizeof(float), send_bytes + 1);
		data.pitch_data_.f =Y;
		#endif

		send_bytes[1] = data.pitch_data_.c[0];
		send_bytes[2] = data.pitch_data_.c[1];
		send_bytes[3] = data.pitch_data_.c[2];
		send_bytes[4] = data.pitch_data_.c[3];

		// char *pa = (char*)&data.pitch_data_.f;
		// printf("first %d, second %d, third %d, forth %d\n", send_bytes[1], send_bytes[2], send_bytes[3], send_bytes[4]);

		send_bytes[5] = data.yaw_data_.c[0];
		send_bytes[6] = data.yaw_data_.c[1];
		send_bytes[7] = data.yaw_data_.c[2];
		send_bytes[8] = data.yaw_data_.c[3];
		#ifdef Plane
        for (int i = 0; i < 4; ++i) 
		{
    	printf("x byte %d: %02X\n", i, send_bytes[1 + i]);
		}
		#endif

		//std::cout << "send pitch" << data.pitch_data_.f << std::endl;
		//std::cout << "send yaw" << data.yaw_data_.f << std::endl;
		#ifndef Plane
		state_ = 0;
		#endif

		//    SET_BIT((data.OnePointFive),6);
		//    CLEAR_BIT((data.OnePointFive),7);
		//    CLEAR_BIT((data.OnePointFive),8);
		
		//1为低位，8为高位
		if (state_ == 0)
		{ // zimiao  001

			SET_BIT((data.OnePointFive), 1);
			CLEAR_BIT((data.OnePointFive), 2);
			CLEAR_BIT((data.OnePointFive), 3);
		}
		else if (state_ == 1)
		{ // dafu   110
			SET_BIT((data.OnePointFive), 3);
			SET_BIT((data.OnePointFive), 2);
			CLEAR_BIT((data.OnePointFive), 1);
		}
		else if (state_ == 2)
		{ // xiaofu   100
			SET_BIT((data.OnePointFive), 3);
			CLEAR_BIT((data.OnePointFive), 2);
			CLEAR_BIT((data.OnePointFive), 1);
		}
		else
		{ // fantuoluo   010

			CLEAR_BIT((data.OnePointFive), 3);
			SET_BIT((data.OnePointFive), 2);
			CLEAR_BIT((data.OnePointFive), 1);
		}

		if (data.is_fire == 1)
		{
			SET_BIT((data.OnePointFive), 4);
		}
		else
		{
			CLEAR_BIT((data.OnePointFive), 4);
			//cout<<"NOT FIRE"<<endl;
		}

		if (data.is_have_armor == 1)
		{
			#ifdef Plane
			send_bytes[9] =1;
            std::cout << "1" << std::endl;
			#endif
			#ifndef Plane
			SET_BIT((data.OnePointFive), 6);
			SET_BIT((data.OnePointFive), 5);
			#endif
		}
		else
		{
			#ifdef Plane
			send_bytes[9] =0;
            std::cout << "0" << std::endl;
			#endif
			#ifndef Plane
			CLEAR_BIT((data.OnePointFive), 6);
			CLEAR_BIT((data.OnePointFive), 5);
			#endif
		}

		//   SET_BIT((data.OnePointFive),2);
		#ifndef Plane
		send_bytes[9] = data.OnePointFive;
		//printf("FIRE:%x\n",data.OnePointFive);
		#endif
		
		// data.time.f = 1;
		send_bytes[10] = data.time.c[0];
		send_bytes[11] = data.time.c[1];
		send_bytes[12] = data.time.c[2];
		send_bytes[13] = data.time.c[3];

		write(fd, send_bytes, 15);
	}

	void DataControler::getData(int fd, Stm32Data &get_data)
	{
		int bytes = 0;
		// uint8_t cde;
		// unsigned char rec_bytes[1024] = {0};
		// 这是干什么的
		
		ioctl(fd, FIONREAD, &bytes); // 1199
		// cout << "bytes      " << bytes << endl;
		if (bytes < DATA_LENGTH)
		{
			return;
		}
		unsigned char *rec_bytes = new unsigned char[bytes + 100]();
		bytes = read(fd, rec_bytes, bytes);

		//
		int FirstIndex = -1;
		int LastIndex = -1;
		bool is_ed = false;
		int ed_count =0;

		//
		for (int i = 0; i < bytes; i++)
		{
			#ifdef Get_Debug //显示串口收到的全部信息
			printf("%x ",rec_bytes[i]);
			#endif

#ifdef Serial_Self_Withering
			//带自凋零机制的旧版串口通信模式
			if (rec_bytes[i] == 0xcd)
			{
				// cout << "head top index" << endl;
				//is_ed = false;
				ed_count =2;
				FirstIndex = i;
			}
			else if (rec_bytes[i] == bb && FirstIndex != -1 && i - FirstIndex == DATA_LENGTH - 1)
			{
				// cout << "tail top index" << endl;
				LastIndex = i;
				break;
			}
			else if(rec_bytes[i] == 0xed)
			{
				ed_count++;
				
				if(ed_count==1)
				{
					exit(0);
				}
				//is_ed = true;
			}
			else
			{
				// cout << "get data fail" << i+1 << endl;
			}
#endif
#ifndef Serial_Self_Withering //不带自凋零机制的串口通信模式
			if (rec_bytes[i] == 0xcd)
			{
				 //cout << "head top index" << endl;
				 //printf("%x\n",rec_bytes[i]);
				FirstIndex = i;
			}
			else if (rec_bytes[i] == 0xdc && FirstIndex != -1 && i - FirstIndex == DATA_LENGTH - 1)
			{
				 //cout << "tail top index" << endl;
				LastIndex = i;
				break;
			}
			else
			{
				// cout << "get data fail" << i+1 << endl;
			}
#endif


		}

		if (FirstIndex != -1 && LastIndex != -1)
		{
			// get_data.IsHave = true;
			get_data.pitch_data_.c[0] = rec_bytes[FirstIndex + 1];
			get_data.pitch_data_.c[1] = rec_bytes[FirstIndex + 2];
			get_data.pitch_data_.c[2] = rec_bytes[FirstIndex + 3];
			get_data.pitch_data_.c[3] = rec_bytes[FirstIndex + 4];

			get_data.yaw_data_.c[0] = rec_bytes[FirstIndex + 5];
			get_data.yaw_data_.c[1] = rec_bytes[FirstIndex + 6];
			get_data.yaw_data_.c[2] = rec_bytes[FirstIndex + 7];
			get_data.yaw_data_.c[3] = rec_bytes[FirstIndex + 8];

			if((get_data.pitch_data_.f) > 10000)
			{
				std::cout << "pitch error" << std::endl;
				exit(0);
				int bit = getBit(get_data.pitch_data_.c[2], 6);
				if(bit == 1)
				{
					CLEAR_BIT(get_data.pitch_data_.c[2],6);
				}

				int bit1 = getBit(get_data.pitch_data_.c[3], 6);
				if(bit1 == 1)
				{
					CLEAR_BIT(get_data.pitch_data_.c[3],6);
				}
			}

			if((get_data.yaw_data_.f) > 10000)
			{
				exit(0);
				std::cout << "yaw error" << std::endl;
				int bit = getBit(get_data.yaw_data_.c[2], 6);
				if(bit == 1)
				{
					CLEAR_BIT(get_data.yaw_data_.c[2],6);
				}

				int bit1 = getBit(get_data.yaw_data_.c[3], 6);
				if(bit1 == 1)
				{
					CLEAR_BIT(get_data.yaw_data_.c[3],6);
				}
			}

			// printf("PITCH1 is %d,%d,%d,%d \n", get_data.pitch_data_.c[0], get_data.pitch_data_.c[1], get_data.pitch_data_.c[2], get_data.pitch_data_.c[3]);
			// printf("YAW1 is %d,%d,%d,%d \n", get_data.yaw_data_.c[0], get_data.yaw_data_.c[1], get_data.yaw_data_.c[2], get_data.yaw_data_.c[3]);

			// printf("first %d, second %d, third %d, forth %d\n", rec_bytes[1], rec_bytes[2], rec_bytes[3], rec_bytes[4]);


			get_data.OnePointFive = rec_bytes[FirstIndex + 9];
			//1为低位，8为高位
			if (getBit(get_data.OnePointFive, 1) == 0)
			{
				get_data.is_rune = 0;
			}
			else if (getBit(get_data.OnePointFive, 1) == 1)
			{
				get_data.is_rune = 1;

			}
/////********************************************************************电控发过来前哨战是否旋转
			if (getBit(get_data.OnePointFive, 4) == 1)
			{
				get_data.is_rotate_skirmish=1;
			}
			else
			{

				get_data.is_rotate_skirmish=0;
			}
			//来自旧代码（lqq）
			// if (getBit(get_data.OnePointFive, 4) == 0)
			// {
			// 	std::cout << "识别红色" << std::endl;
			// 	get_data.color_ = false;
			// }
			// else
			// {
			// 	std::cout << "识别蓝色" << std::endl;
			// 	get_data.color_ = true;
			// }

			if (getBit(get_data.OnePointFive, 5) == 0)
			{
				get_data.pitch_data_.f = get_data.pitch_data_.f;
			}
			else
			{
				get_data.pitch_data_.f = -get_data.pitch_data_.f;
			}

			if (getBit(get_data.OnePointFive, 6) == 0 && getBit(get_data.OnePointFive, 8) == 0)
			{
				get_data.is_aim = false;
			}
			else
			{
				get_data.is_aim = true;
			}

			if (getBit(get_data.OnePointFive, 7) == 0)
			{
				get_data.yaw_data_.f = get_data.yaw_data_.f;
			}
			else
			{
				get_data.yaw_data_.f = -get_data.yaw_data_.f;
			}
									
			get_data.time.c[0] = rec_bytes[FirstIndex + 10];
			get_data.time.c[1] = rec_bytes[FirstIndex + 11];
			get_data.time.c[2] = rec_bytes[FirstIndex + 12];
			get_data.time.c[3] = rec_bytes[FirstIndex + 13];

			// 接收电控发来的弹速
			get_data.init_firing_rate = rec_bytes[FirstIndex + 14];

			//get_data.IsHave = true;
			cout<<"消息接收完成，时间："<<get_data.time.f<<endl;
			get_data.is_get = true;
			//free(rec_bytes);
		}
		else
		{
			cout << "消息接收失败" << endl;//眼不见心不烦
			// get_data.IsHave = false;
			get_data.is_get = false;
		}

		return;
	}

}
#endif

#ifdef Inter_Vis_Ctl
Stm32Data stm32data;             // 电控数据
VisionData visiondata;           // 视觉数据
Stm32Data stm32data_temp;        // 时钟同步缓冲区
std::deque<Stm32Data> MCU_data_; // 时钟同步队列
unsigned int mcu_size_ = 12;     // 队列长度
void Send_Receive(int sockfd)
{
    int iret;
    // 向电控发送数据
    if ((iret = send(sockfd, visiondata.DATA, sizeof(visiondata.DATA), 0)) == -1) // send data to server
    {
        close(sockfd);
        // perror("send");
        return;
    }
    std::cout << "\033[34mSend: \033[0m" << visiondata.yaw_data_ << " " << visiondata.pitch_data_ << std::endl;
    toVofa.visionYaw = visiondata.yaw_data_;
    toVofa.visionPitch = visiondata.pitch_data_;
    // 接受电控的数据
    if ((iret = recv(sockfd, stm32data_temp.DATA, sizeof(stm32data_temp.DATA), 0)) <= 0) // receive server's reply
    {
        printf("iret=%d\n", iret);
        close(sockfd);
        return;
    }
    // 时钟同步数据准备
    if (MCU_data_.size() < mcu_size_)
    {
        MCU_data_.push_back(stm32data_temp);
    }
    else
    {
        MCU_data_.pop_front();
        MCU_data_.push_back(stm32data_temp);
    }
    std::cout << "\033[31mReceive: \033[0m" << stm32data.YAW_DATA << " " << stm32data.PITCH_DATA << " " << stm32data.time << std::endl;
    toVofa.RuiYaw = stm32data.YAW_DATA;
    toVofa.RuiPitch = stm32data.PITCH_DATA;
}
#endif