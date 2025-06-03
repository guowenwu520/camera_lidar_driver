#include "TanWayLidarManager.h"
#include <iostream>
#include <chrono>

using namespace tanway;

class TanWayLidarManager::LidarObserverImpl : public ILidarObserver {
public:
    void OnPointCloud(const LidarInfo &lidarInfo, const UserPointCloud &pointCloud) override {
        std::cout << "width:" << pointCloud.width
                  << " height:" << pointCloud.height
                  << " point cloud size: " << pointCloud.points.size() << std::endl;
    }

    void OnIMU(const LidarInfo &, const IMUData &) override {
        // 可按需添加 IMU 数据处理
    }

    void OnException(const LidarInfo &, const Exception &e) override {
        if (e.GetErrorCode() > 0)
            std::cout << "[Error Code]: " << e.GetErrorCode() << " -> " << e.ToString() << std::endl;
        if (e.GetTipsCode() > 0)
            std::cout << "[Tips Code]: " << e.GetTipsCode() << " -> " << e.ToString() << std::endl;
    }

    void OnDeviceInfoFrame(const LidarInfo &, const DeviceInfoFrame &) override {}
    void OnParsePcapProcess(const LidarInfo &, int, uint64_t, uint64_t) override {}
};

TanWayLidarManager::TanWayLidarManager(const std::string& local_ip,
                                       const std::string& lidar_ip,
                                       uint16_t pointcloud_port,
                                       uint16_t config_port,
                                       const std::string& algo_config_path)
    : observer_(std::make_shared<LidarObserverImpl>()), running_(false)
{
    lidar_ = ILidarDevice::Create(lidar_ip, local_ip, pointcloud_port, config_port,
                                  observer_.get(), LT_FocusB2_B3_MP);
    algo_ = ILidarAlgo::Create(LT_FocusB2_B3_MP, algo_config_path);
    lidar_->SetLidarAlgo(algo_.get());
}

TanWayLidarManager::~TanWayLidarManager() {
    Stop();
    if (toggle_thread_.joinable()) {
        toggle_thread_.join();
    }
}

void TanWayLidarManager::Start() {
    if (lidar_) {
        lidar_->Start();
        std::cout << "TanWayLidar started." << std::endl;
    }
}

void TanWayLidarManager::Stop() {
    if (lidar_) {
        lidar_->Stop();
        std::cout << "TanWayLidar stopped." << std::endl;
    }
}

void TanWayLidarManager::RunWithAutoToggle(int interval_seconds) {
    running_ = true;
    toggle_thread_ = std::thread([this, interval_seconds]() {
        bool toggle = false;
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(interval_seconds));
            if (toggle) {
                Start();
            } else {
                Stop();
            }
            toggle = !toggle;
        }
    });
}
