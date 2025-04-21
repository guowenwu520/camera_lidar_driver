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
	std::string ip = "192.168.0.2";				// device ip
	const int port = 2469;						// device port
	benewake::BenewakeLidar lidar(ip, port); 	// init lidar driver

	bool run = true;
	benewake::BwPointCloud::Ptr pointCloud;
	char fileName[100];
	int nFrame = 0;
	bool succ = false;

	uint32_t delay_time = 0;
	float pkg_loss_rate = 0, frame_loss_rate = 0;

	std::string ip_device = "192.168.1.22";			// local ip
	const int port_device = 4321;					// local port
	succ = lidar.startOnlyReceiver(ip_device, port_device, 0x00, 0x01, true, false, 0); // start lidar's measurement
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
		lidar.getDataTransmissionState(delay_time, pkg_loss_rate, frame_loss_rate);
		int points_amount = pointCloud->points.size();
		std::cout << "Frame " << nFrame << " point cloud size: " << points_amount << std::endl;
		std::cout << "  transmission stat: " << "dt " << delay_time << " pkg loss " 
					<< pkg_loss_rate << "\% frame loss " << frame_loss_rate << "\%\n"; 
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
}
