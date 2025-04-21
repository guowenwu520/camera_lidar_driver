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
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        recv = getchar();
        return recv;
    }
    tcsetattr(0,TCSANOW,&stored_settings);
    return 'n';
}
#endif // _WIN32

void displayCallback(benewake::BwPointCloud::Ptr _pointcloud, int _frame_id, void* _pData)
{
	int points_amount = _pointcloud->points.size();
	std::cout << "Frame " << _frame_id << " point cloud size: " << points_amount << std::endl;
	if (points_amount > 0)
	{
		/*
		* do some work here
		*/
		printf("  sampling start at %llu.%09u, end at %llu.%09u\n", 
			_pointcloud->points[0].timestamp_s, _pointcloud->points[0].timestamp_ns,
			_pointcloud->points[points_amount - 1].timestamp_s, _pointcloud->points[points_amount - 1].timestamp_ns);
		// copy data to ouput pointer
		benewake::BwPointCloud::Ptr* pointCloud = static_cast<benewake::BwPointCloud::Ptr*>(_pData);
		(*pointCloud)->points = _pointcloud->points;
	}
}

void heartbeatCallback(benewake::SYS_INFO _info, void* _pData)
{
	int code = benewake::BW_GET_SYSTEM_STATUS_CODE(_info);
	if (code == BW_COMM_ERROR)
		std::cout << "ERROR: communication error!" << std::endl;
	else if (code == BW_SYS_ERROR)
		std::cout << "ERROR: device system error!" << std::endl;
	else if (code == BW_SYS_WARNING)
		std::cout << "WARNING: device system warning!" << std::endl;
	else if (code == BW_SYS_BUSY)
		std::cout << "WARNING: device system busy!" << std::endl;
}

int main()
{
	std::string ip = "192.168.0.2";
	benewake::BenewakeLidar lidar(ip, 2469); // init lidar driver
	bool run = true;
	char fileName[100];
	int nFrame = 0;

	// regist callback
	benewake::BwPointCloud::Ptr pointCloud = std::make_shared<benewake::BwPointCloud>();
	lidar.registPointCloudCallback(displayCallback, (void*)&pointCloud);
	lidar.registHeartBeatCallback(heartbeatCallback, NULL);

#ifdef _WIN32
	Sleep(3000);
#else
	usleep(3000000);
#endif // _WIN32

	bool succ = false;
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
	if (!succ) {
		std::cerr << "start error!" << std::endl;
		run = false;
	}
	benewake::SYS_INFO sys_info;
	while (run)
	{
		/*
		* Do some work ...
		*/
		std::cout << "-->Outside loop point cloud size: " << pointCloud->points.size() << std::endl;
#ifdef _WIN32
		Sleep(140);
#else
		usleep(140000);
#endif // _WIN32

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
	std::cout << "lidar stop!" << std::endl;
	lidar.stop(); // stop lidar's measurement
	system("pause");
}
