#include "TanwayLidarManager.h"
#include <chrono>
#include <fstream>
#include <iomanip>

using namespace tanway;

TanwayLidarManager::TanwayLidarManager(int client_socket,FileManager fileManager,
                                       const std::string &local_ip,
                                       const std::string &lidar_ip,
                                       const std::string &algo_config_path)
    : fileManager(fileManager),
      local_ip(local_ip),
      lidar_ip(lidar_ip),
      algo_config_path(algo_config_path),
      save_dir(fileManager.get_64_lidar_save_path()),
      client_socket(client_socket)
{
    thread_pool = std::make_shared<ThreadPool>(2); // 可根据需要调整线程数量
}

TanwayLidarManager::~TanwayLidarManager()
{
    stop();
}

bool TanwayLidarManager::initialize()
{
    lidar = ILidarDevice::Create(lidar_ip.c_str(), local_ip.c_str(), 5600, 5700, this, LT_FocusB2_B3_MP);
    if (!lidar)
    {
        std::cerr << "[TanwayLidarManager] Failed to create lidar device!" << std::endl;
        return false;
    }

    algo = ILidarAlgo::Create(LT_FocusB2_B3_MP, algo_config_path);
    if (!algo)
    {
        std::cerr << "[TanwayLidarManager] Failed to create lidar algorithm!" << std::endl;
        return false;
    }

    lidar->SetLidarAlgo(algo.get());
    lidar_ready = true;
    return true;
}

void TanwayLidarManager::start()
{
    if (lidar_ready)
    {
        lidar->Start();
        std::cout << "[TanwayLidarManager] Lidar started." << std::endl;
    }
}

void TanwayLidarManager::stop()
{
    if (lidar_ready)
    {
        lidar->Stop();
        std::cout << "[TanwayLidarManager] Lidar stopped." << std::endl;
    }
}

bool TanwayLidarManager::hasLidar() const
{
    return lidar_ready;
}

void TanwayLidarManager::OnPointCloud(const LidarInfo &info, const UserPointCloud &cloud)
{
    auto task = [cloud, this]() {
        std::ostringstream filename;
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);

        filename << save_dir << "/pointcloud_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".txt";

        std::ofstream out(filename.str());
        if (!out.is_open())
        {
            std::cerr << "[TanwayLidarManager] Failed to open file: " << filename.str() << std::endl;
            return;
        }

        for (const auto &pt : cloud.points)
        {
            out << pt.x << "," << pt.y << "," << pt.z << "\n";
        }

        out.close();
        std::cout << "[TanwayLidarManager] Saved point cloud: " << filename.str() << std::endl;
    };

    thread_pool->enqueue(task);
}

void TanwayLidarManager::OnException(const LidarInfo &info, const Exception &e)
{
    if (e.GetErrorCode() > 0)
        std::cerr << "[TanwayLidarManager] Error: " << e.GetErrorCode() << " -> " << e.ToString() << std::endl;
    if (e.GetTipsCode() > 0)
        std::cerr << "[TanwayLidarManager] Tip: " << e.GetTipsCode() << " -> " << e.ToString() << std::endl;
}

void TanwayLidarManager::OnIMU(const LidarInfo &lidarInfo, const IMUData &imu) {
// std::cout << "IMU data callback:" << std::endl
//       << "  Angular velocity [rad/s]: "
//       << "X: " << std::fixed << std::setprecision(6)
//       << imu.angular_velocity[0] << ", Y: " << std::fixed
//       << std::setprecision(6) << imu.angular_velocity[1]
//       << ", Z: " << std::fixed << std::setprecision(6)
//       << imu.angular_velocity[2] << std::endl
//       << "  Linear acceleration [g]: "
//       << "X: " << std::fixed << std::setprecision(6)
//       << imu.linear_acceleration[0] << ", Y: " << std::fixed
//       << std::setprecision(6) << imu.linear_acceleration[1]
//       << ", Z: " << std::fixed << std::setprecision(6)
//       << imu.linear_acceleration[2] << std::endl;

  }
void TanwayLidarManager::OnDeviceInfoFrame(const LidarInfo &info, const DeviceInfoFrame &deviceInfoFrame)
{
    // 可选实现或留空
}

void TanwayLidarManager::OnParsePcapProcess(const LidarInfo &info, int process, uint64_t frame, uint64_t stamp)
{
    // 可选实现或留空
}
