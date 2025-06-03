#ifndef TANWAY_LIDAR_MANAGER_H
#define TANWAY_LIDAR_MANAGER_H

#include "ILidarDevice.h"
#include "ILidarAlgo.h"
#include <memory>
#include <string>
#include <thread>
#include <atomic>

class TanWayLidarManager {
public:
    TanWayLidarManager(const std::string& local_ip="192.168.111.204",
                       const std::string& lidar_ip="192.168.111.51",
                       uint16_t pointcloud_port=5600,
                       uint16_t config_port=5700,
                       const std::string& algo_config_path="/root/workspace/CollectionControlServer/tanwaylidar/config/algo_table.json");

    ~TanWayLidarManager();

    void Start();
    void Stop();
    void RunWithAutoToggle(int interval_seconds = 3);

private:
    class LidarObserverImpl;
    std::shared_ptr<LidarObserverImpl> observer_;
    std::shared_ptr<tanway::ILidarDevice> lidar_;
    std::shared_ptr<tanway::ILidarAlgo> algo_;
    std::atomic<bool> running_;
    std::thread toggle_thread_;
};

#endif  // TANWAY_LIDAR_MANAGER_H
