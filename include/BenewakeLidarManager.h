#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "benewake_lidar_driver.h"
#include "SocketServer.h"
#include "SocketClient.h"
#include "File_uitl.h"
#include "Globals.h"
#include "ThreadPool.h"

class BenewakeLidarManager
{
public:
    BenewakeLidarManager(FileManager fileManager, const std::string &ip = "192.168.0.2", int port = 2469);

    bool initialize();
    bool hasLidar() const;
    void start();
    void stop();
    std::string getSave_Path();

private:
    std::string lidar_ip;
    int lidar_port;
    std::string dir;
    std::shared_ptr<benewake::BenewakeLidar> lidar;
    std::shared_ptr<ThreadPool> pool;
    FileManager fileManager;
    bool lidar_present = false;
    bool save_enabled = false;

    void main_loop();
};
