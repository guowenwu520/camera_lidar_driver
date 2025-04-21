// vs2017.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

//#include "pch.h"
#define NOMINMAX
#include <iostream>
#include <string>
#include "benewake_lidar_driver.h"

#ifdef _WIN32
#include <conio.h>
#else
#include <termio.h>

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

int main()
{
	bool succ = false;
	std::string ip = "192.168.0.2";

	// *****************************************************************
	// * If lidar's multicast is not enabled, should enable lidar multicast mode first using following codes
	// * ---------------------------------------------------------------
	//benewake::BenewakeHornX2 lidar(ip, 2469, benewake::UDPType::NORMAL);
	//succ = lidar.setMulticastStatus(true);
	//if (!succ)
	//{
	//	std::cerr << "set lidar to multicast mode failed!" << std::endl;
	//	return 0;
	//}
	//succ = lidar.updateFlash();
	//if (!succ)
	//{
	//	std::cerr << "save lidar config failed!" << std::endl;
	//	return 0;
	//}
	//succ = lidar.reboot();
	//if (!succ)
	//{
	//	std::cerr << "reboot lidar failed!" << std::endl;
	//	return 0;
	//}
	//std::cout << "Success! Lidar will be multicast mode after reboot." << std::endl;
	//return 1;
	// *****************************************************************

	bool run = true;
	benewake::BenewakeLidar lidar(ip, 2469, benewake::UDPType::MULTICAST);
	benewake::BwPointCloud::Ptr pointCloud;
	char fileName[100];
	int nFrame = 0;

#ifdef _WIN32
	Sleep(3000);
#else
	usleep(3000000);
#endif // _WIN32
	// get device info if needed
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
	//else
	//{
	//	std::cout << "error: get device information failed!\n";
	//}
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

	succ = lidar.start(); // start lidar's measurement
	if (!succ)
	{
		std::cerr << "start error!" << std::endl;
		run = false;
	}
	benewake::SYS_INFO sys_info;
	while (run)
	{
		succ = lidar.getData(pointCloud, nFrame, sys_info); // get a frame of point cloud
		if (!succ)
		{
			int err_code = benewake::BW_GET_SYSTEM_STATUS_CODE(sys_info);
			std::cerr << "Error occured! Error code: " << err_code << std::endl;
		}
		int points_amount = pointCloud->points.size();
		std::cout << "Frame " << nFrame << " point cloud size: " << points_amount << std::endl;
		if (points_amount > 0)
		{
			// note that when set timestamp output format as 3 - GPS&PPS, time's unit is not second. See documents for details.
			printf("  sampling start at %llu.%09u, end at %llu.%09u\n",
				   pointCloud->points[0].timestamp_s, pointCloud->points[0].timestamp_ns,
				   pointCloud->points[points_amount - 1].timestamp_s, pointCloud->points[points_amount - 1].timestamp_ns);

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
		char in = getInput();
		{
			if (in == 'Q' || in == 'q')
				run = false;
			else if (in == 'S' || in == 's')
			{
			}
		}
#endif //_WIN32
	}
	lidar.stop(); // stop lidar's measurement
}
