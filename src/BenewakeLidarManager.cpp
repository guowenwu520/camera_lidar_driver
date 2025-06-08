#include "BenewakeLidarManager.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>

BenewakeLidarManager::BenewakeLidarManager(FileManager fileManager, const std::string &ip, int port)
    : lidar_ip(ip), lidar_port(port), save_enabled(false), fileManager(fileManager)
{
}

bool BenewakeLidarManager::initialize()
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

bool BenewakeLidarManager::hasLidar() const
{
    return lidar_present;
}

void BenewakeLidarManager::start()
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

void BenewakeLidarManager::stop()
{
    Config::running = false;
    if (lidar)
        lidar->stop();
}

void BenewakeLidarManager::main_loop()
{
    int cur_frame = 0;
    benewake::BwPointCloud::Ptr pointCloud;
    benewake::SYS_INFO sys_info;
    int nFrame = 0;
    Config::running = true;

    dir = fileManager.get_256_lidar_save_path();
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
