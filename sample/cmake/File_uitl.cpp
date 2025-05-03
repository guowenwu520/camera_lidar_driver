#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pwd.h>
#include <termio.h>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <iomanip>
#include <regex>
#include "Globals.h"
#include "benewake_lidar_driver.h"
class FileManager
{
public:
    const std::string root_path = "/root/workspace/save_path/";

    std::string getRootPath()
    {
        return root_path;
    }

    std::vector<std::string> getFileChildPaths()
    {
        return get_subdirectories(root_path);
    }

    void setSavePath(std::string name)
    {
        // select_path = root_path + name + "/Sequences";
        Config::select_path = name + "/Sequences";
        // std::cout << "select + path ;" << Config::select_path << std::endl;
    }

    int getPathCount(std::string dir)
    {
        dir=root_path+dir;
        std::vector<std::string> current_files = get_subdirectories(dir);
        int number = 0;
        for (size_t i = 0; i < current_files.size(); ++i)
        {
            number = std::max(number,std::stoi(current_files[i]));
        }
        return number+1;
    }

    std::string getSavePath()
    {
        return Config::select_path;
    }

    bool createDirectory(const std::string &namefile)
    {
        std::string dir = root_path + namefile;
        std::string cmd = "mkdir -p " + dir;
        int ret = system(cmd.c_str());
        if (ret != 0)
        {
            std::cerr << "Failed to create directory using mkdir -p: " << dir << std::endl;
            return false;
        }
        return true;
    }

    bool deleteFile(const std::string &filename)
    {
        std::string path = root_path + filename;
        std::string cmd = "rm -r -f " + path;
        int ret = system(cmd.c_str());
        if (ret != 0)
        {
            std::cerr << "Failed to create directory using mkdir -p: " << path << std::endl;
            return false;
        }
        return true;
    }

    std::vector<std::string> get_usb_mounts()
    {
        std::vector<std::string> usb_paths;

        // 获取当前用户名对应的挂载目录
        const char *home = getenv("HOME");
        std::string media_path = std::string("/media/") + (home ? std::string(getpwuid(getuid())->pw_name) : "user");

        DIR *dir = opendir(media_path.c_str());
        if (!dir)
        {
            // perror(("无法打开目录: " + media_path).c_str());
            return usb_paths;
        }

        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string name = entry->d_name;
            if (name == "." || name == "..")
                continue;

            std::string full_path = media_path + "/" + name;
            struct stat st;
            if (stat(full_path.c_str(), &st) == 0 && S_ISDIR(st.st_mode))
            {
                usb_paths.push_back(full_path);
            }
        }

        closedir(dir);
        return usb_paths;
    }

    bool isHasUsb()
    {
        auto usb_paths = get_usb_mounts();
        if (usb_paths.empty())
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    void savePointCloudAsKITTI(const benewake::BwPointCloud::Ptr &cloud, int number, std::string dir, int frameNum)
    {
        if (number < 10)
            dir = dir + "/0" + std::to_string(number) + "/velodyne";
        else
            dir = dir + "/" + std::to_string(number) + "/velodyne";

        createDirectory(dir);
        dir = root_path + dir;
        std::ostringstream oss;
        oss << dir << "/frame_" << std::setw(6) << std::setfill('0') << frameNum << ".bin";
        std::ofstream ofs(oss.str(), std::ios::out | std::ios::binary);

        if (!ofs.is_open())
        {
            std::cerr << "Failed to open file for writing: " << oss.str() << std::endl;
            return;
        }

        for (const auto &pt : cloud->points)
        {
            float data[4] = {pt.x, pt.y, pt.z, pt.intensity};
            ofs.write(reinterpret_cast<const char *>(data), sizeof(float) * 4);
        }

        ofs.close();
        std::cout << "Saved KITTI point cloud: " << oss.str() << std::endl;
    }

private:
    static std::vector<std::string> get_subdirectories(const std::string &path)
    {
        std::vector<std::string> folders;
        DIR *dir = opendir(path.c_str());
        if (dir == nullptr)
        {
            perror("opendir failed");
            return folders;
        }

        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string name = entry->d_name;

            if (name == "." || name == "..")
                continue;

            std::string full_path = path + "/" + name;

            struct stat st;
            if (stat(full_path.c_str(), &st) == 0 && S_ISDIR(st.st_mode))
            {
                folders.push_back(name);
            }
        }

        closedir(dir);
        return folders;
    }
};

#endif // FILE_MANAGER_H
