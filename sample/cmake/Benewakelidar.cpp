#pragma once
#include <iostream>
#include <string>
#include "benewake_lidar_driver.h"
#include <termio.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <queue>
#include <thread>
#include <mutex>
#include "SocketServer.h"
#include "SocketClient.h"
#include <thread>
#include <chrono>
#include "thread_pool.cpp"
#include "File_uitl.cpp"
#include "Globals.h"
#include <condition_variable>

class BenewakeLidarManager
{
public:
    BenewakeLidarManager(FileManager fileManager, const std::string &ip = "192.168.0.2", int port = 2469)
        : lidar_ip(ip), lidar_port(port), save_enabled(false), fileManager(fileManager)
    {
    }

    bool initialize()
    {
        lidar = std::make_shared<benewake::BenewakeLidar>(lidar_ip, lidar_port);
        std::string version, version_fpga, sn;
        int total_num, line_num, channel_num;

        std::cout << "Trying to get device info...\n";
        if (lidar->getDeviceInformation(version, version_fpga, total_num, line_num, channel_num, sn))
        {
            std::cout << "[LIDAR] version: " << version << "\nFPGA: " << version_fpga
                      << "\nTotal points: " << total_num
                      << "\nLine points: " << line_num
                      << "\nChannels: " << channel_num
                      << "\nSN: " << sn << std::endl;
            lidar_present = true;
            return true;
        }
        else
        {
            std::cerr << "[ERROR] Failed to get LIDAR info.\n";
            lidar_present = false;
            return false;
        }
    }

    bool hasLidar() const
    {
        return lidar_present;
    }

    void start()
    {
        if (!lidar_present)
        {
            std::cerr << "[ERROR] No LIDAR detected. Aborting start().\n";
            return;
        }

        unsigned int num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0)
            num_threads = 4;
        pool = std::make_shared<ThreadPool>(num_threads);

        lidar->stop();
        std::this_thread::sleep_for(std::chrono::seconds(3));

        if (!lidar->start())
        {
            std::cerr << "[ERROR] Failed to start LIDAR\n";
            return;
        }

        main_loop();
        lidar->stop();
    }

    void stop()
    {
        Config::running = false;
        if (lidar)
            lidar->stop();
    }
    std::string getSave_Path()
    {
        return dir;
    }

private:
    std::string lidar_ip;
    int lidar_port;
    std::string dir;
    std::shared_ptr<benewake::BenewakeLidar> lidar;
    std::shared_ptr<ThreadPool> pool;
    FileManager fileManager;
    bool lidar_present = false;
    bool save_enabled = false;

    void main_loop()
    {
        int cur_frame = 0;
        benewake::BwPointCloud::Ptr pointCloud;
        benewake::SYS_INFO sys_info;
        int nFrame = 0;
        Config::running = true;
        // 注意路径为空
        dir = fileManager.getSavePath();
        if (dir.length() <= 0)
        {
            std::cout << "select save_path" << std::endl;
            return;
        }
        int number = fileManager.getPathCount(dir);
        while (Config::running)
        {
            bool ok = lidar->getData(pointCloud, nFrame, sys_info);
            if (!ok)
            {
                int err = benewake::BW_GET_SYSTEM_STATUS_CODE(sys_info);
                std::cerr << "[ERROR] LIDAR data failed, code: " << err << std::endl;
                continue;
            }

            if (pointCloud->points.size() > 0)
            {
                pool->enqueue([=]
                              { fileManager.savePointCloudAsKITTI(pointCloud, number, dir, cur_frame); });
                cur_frame++;
            }
        }
        Config::running = false;
    }
};