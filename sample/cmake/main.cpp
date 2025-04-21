// vs2017.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// #include "pch.h"
#define NOMINMAX
#include <iostream>
#include <string>
#include "benewake_lidar_driver.h"

#ifdef _WIN32
#include <conio.h>
#else
#include <termio.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <queue>
#include <thread>
#include <mutex>
#include "thread_pool.cpp"
#include <condition_variable>

std::queue<std::pair<benewake::BwPointCloud::Ptr, int>> saveQueue;
std::mutex queueMutex;
bool saveThreadRunning = true;
std::string saveDir = "./saved_frames"; // 默认路径

// 创建目录（如果不存在）

void createDirectory(const std::string &dir)
{
	std::string cmd = "mkdir -p "+dir;
    int ret = system(cmd.c_str());
    if (ret != 0) {
        std::cerr << "Failed to create directory using mkdir -p: " << dir << std::endl;
    }
}
void savePointCloudAsKITTI(const benewake::BwPointCloud::Ptr &cloud, std::string dir, int number,int frameNum)
{
	if(number<10)
     	dir=dir+ "/0"+std::to_string(number)+"/velodyne";
	else 
     	dir=dir+ "/"+std::to_string(number)+"/velodyne";
	createDirectory(dir);
	std::ostringstream oss;
	oss << dir << "/frame_" << std::setw(6) << std::setfill('0') << frameNum << ".bin";
	std::ofstream ofs(oss.str(), std::ios::out | std::ios::binary);

	if (!ofs.is_open())
	{
		std::cerr << "Failed to open file for writing: " << oss.str() << std::endl;
		return;
	}

	for (const auto &pt : cloud->points)
	{
		float data[4] = {pt.x, pt.y, pt.z, pt.intensity};
		ofs.write(reinterpret_cast<const char *>(data), sizeof(float) * 4);
	}

	ofs.close();
	std::cout << "Saved KITTI point cloud: " << oss.str() << std::endl;
}

char getInput()
{
	fd_set rfds;
	struct timeval tv;
	char recv = '\0';

	FD_ZERO(&rfds);
	FD_SET(0, &rfds);
	tv.tv_sec = 0;
	tv.tv_usec = 5;

	struct termios new_settings;
	struct termios stored_settings;
	tcgetattr(0, &stored_settings);
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	tcgetattr(0, &stored_settings);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &new_settings);
	if (select(1, &rfds, NULL, NULL, &tv) > 0)
	{
		recv = getchar();
		return recv;
	}
	tcsetattr(0, TCSANOW, &stored_settings);
	return 'n';
}
#endif // _WIN32

int main(int argc, char **argv)
{
	if (argc > 1)
	{
		saveDir = argv[1];
		std::cout << "Custom save path set to: " << saveDir << std::endl;
	}
	else
	{
		std::cout << "No path specified, using default path: " << saveDir << std::endl;
	}
	saveDir += "/Sequences";

	createDirectory(saveDir);
	std::string ip = "192.168.0.2";
	benewake::BenewakeLidar lidar(ip, 2469); // init lidar driver
	bool run = true;
	benewake::BwPointCloud::Ptr pointCloud;
	char fileName[100];
	int nFrame = 0;

#ifdef _WIN32
	Sleep(3000);
#else
	usleep(3000000);
#endif // _WIN32

	bool succ = false;
	bool save_enabled = false;

	unsigned int numThreads = std::thread::hardware_concurrency();
	if (numThreads == 0)
		numThreads = 4; // fallback
	// 创建一个线程池实例，假设使用4个线程
	auto pool = std::make_shared<ThreadPool>(numThreads);

	// get device info
	std::string version, version_fpga, sn;
	int total_num, line_num, channel_num;
	std::cout << "Try get device information ...\n";
	bool status = lidar.getDeviceInformation(version, version_fpga, total_num, line_num, channel_num, sn);
	if (status)
	{
		std::cout << "  version: " << version << std::endl;
		std::cout << "  FPGA: " << version_fpga << std::endl;
		std::cout << "  Total points (simgle channel): " << total_num << std::endl;
		std::cout << "  Line points: " << line_num << std::endl;
		std::cout << "  Channel amount: " << channel_num << std::endl;
		std::cout << "  SN: " << sn << std::endl;
	}
	// else
	//{
	//	std::cout << "error: get device information failed!\n";
	// }
	succ = lidar.stop();
	if (!succ)
	{
		std::cerr << "stop error!" << std::endl;
		run = false;
	}

#ifdef _WIN32
	Sleep(3000);
#else
	usleep(3000000);
#endif // _WIN32
	uint32_t delay_time = 0;
	float pkg_loss_rate = 0, frame_loss_rate = 0;

	succ = lidar.start(); // start lidar's measurement
	if (!succ)
	{
		std::cerr << "start error!" << std::endl;
		run = false;
	}
	benewake::SYS_INFO sys_info;
	int number=0;
	int cur_count_frame=0;
	while (run)
	{
		succ = lidar.getData(pointCloud, nFrame, sys_info); // get a frame of point cloud
		if (!succ)
		{
			int err_code = benewake::BW_GET_SYSTEM_STATUS_CODE(sys_info);
			std::cerr << "Error occured! Error code: " << err_code << std::endl;
		}
		lidar.getDataTransmissionState(delay_time, pkg_loss_rate, frame_loss_rate);
		int points_amount = pointCloud->points.size();
		// std::cout << "Frame " << nFrame << " point cloud size: " << points_amount << std::endl;
		// std::cout << "  transmission stat: " << "dt " << delay_time << " pkg loss "
		// 		  << pkg_loss_rate << "\% frame loss " << frame_loss_rate << "\%\n";
		if (points_amount > 0)
		{
			// note that when set timestamp output format as 3 - GPS&PPS, time's unit is not second. See documents for details.
			// printf("  sampling start at %llu.%09u, end at %llu.%09u\n",
			//    pointCloud->points[0].timestamp_s, pointCloud->points[0].timestamp_ns,
			//    pointCloud->points[points_amount - 1].timestamp_s, pointCloud->points[points_amount - 1].timestamp_ns);

			/*
			 * Do some work ...
			 */
		}

#ifdef _WIN32
		if (_kbhit())
		{
			char key = _getch();
			if (key == 'Q' || key == 'q')
				run = false;
			else if (key == 'S' || key == 's')
			{
			}
		}
#else
		if (save_enabled)
		{
			// 将保存点云任务放入线程池
			pool->enqueue([=]
						  { savePointCloudAsKITTI(pointCloud, saveDir,number, cur_count_frame); });
			cur_count_frame++;
		}

		char in = getInput();
		if (in == 'Q' || in == 'q')
		{
			run = false;
		}
		else if (in == 'S' || in == 's')
		{
			save_enabled = true;
			std::cout << "[INFO] Start saving frames." << std::endl;
		}
		else if (in == 'E' || in == 'e')
		{
			number++;
			cur_count_frame=0;
			save_enabled = false;
			std::cout << "[INFO] Stop saving frames." << std::endl;
		}

#endif //_WIN32
	}

	lidar.stop(); // stop lidar's measurement
}
