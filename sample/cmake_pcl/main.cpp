// vs2017.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

//#include "pch.h"
#define NOMINMAX
#include <iostream>
#include <string>
#include "benewake_lidar_driver.h"
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/make_shared.hpp>

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
	std::string ip = "192.168.1.10";
	benewake::BenewakeLidar lidar(ip, 2469); // init lidar driver

#ifdef _WIN32
	Sleep(3000);
#else
	usleep(3000000);
#endif // _WIN32

	bool run = true;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	benewake::BwPointCloud::Ptr pointCloud;
	char fileName[100];
	int nFrame = 0;
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = std::make_shared<pcl::visualization::PCLVisualizer>();
	viewer->addCoordinateSystem(2.0);
	viewer->setCameraPosition(-40.0, 0.0, 40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	bool succ = false;
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
	else
	{
		std::cout << "error: get device information failed!\n";
	}
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
	pcl::PointXYZI pt;
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color(cloud, "intensity");
	viewer->addPointCloud(cloud, color, "cloud");
	while (run)
	{
		succ = lidar.getData(pointCloud, nFrame, sys_info); // get a frame of point cloud
		if (!succ)
		{
			int err_code = benewake::BW_GET_SYSTEM_STATUS_CODE(sys_info);
			std::cerr << "Error occured! Error code: " << err_code << std::endl;
		}
		int points_amount = pointCloud->points.size();
		if (points_amount > 0)
		{
			//std::cout << "Frame " << nFrame << " point cloud size: " << pointCloud->points.size() << std::endl;
			// note that when set timestamp output format as 3 - GPS&PPS, time's unit is not second. See documents for details.
			//printf("  sampling start at %llu.%09u, end at %llu.%09u\n",
			//	pointCloud->points[0].timestamp_s, pointCloud->points[0].timestamp_ns,
			//	pointCloud->points[points_amount - 1].timestamp_s, pointCloud->points[points_amount - 1].timestamp_ns);

			/*
			* Do some work ...
			*/
			cloud->resize(points_amount);
			for (int i = 0; i < points_amount; i++)
			{
				cloud->points[i].x = pointCloud->points[i].x;
				cloud->points[i].y = pointCloud->points[i].y;
				cloud->points[i].z = pointCloud->points[i].z;
				cloud->points[i].intensity = pointCloud->points[i].intensity;
			}

			//viewer.removePointCloud("cloud");
			pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fieldColor(cloud, "intensity");
			viewer->updatePointCloud(cloud, fieldColor, "cloud");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
			viewer->spinOnce(1);
		}

#ifdef _WIN32
		if (_kbhit())
		{
			char key = _getch();
			if (key == 'Q' || key == 'q')
				run = false;
			else if (key == 'S' || key == 's')
			{
				sprintf(fileName, "benewake_Horn_X2_pc%.6d.pcd", nFrame);
				if (cloud->size() > 0)
				{
					pcl::io::savePCDFileASCII(fileName, *cloud); // save a frame of point cloud to pcd file
					std::cout << "pcd file saved: " << fileName << std::endl;
				}
			}
		}
#else
		char in = getInput();
		{
			if (in == 'Q' || in == 'q')
				run = false;
			else if (in == 'S' || in == 's')
			{
				sprintf(fileName, "benewake_Horn_X2_pc%.6d.pcd", nFrame);
				if (cloud->size() > 0)
				{
					pcl::io::savePCDFileASCII(fileName, *cloud); // save a frame of point cloud to pcd file
					std::cout << "pcd file saved: " << fileName << std::endl;
				}
			}
		}
#endif //_WIN32
	}
	lidar.stop(); // stop lidar's measurement
}
// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started:
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
