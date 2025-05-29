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
        Config::select_path = name + "/";
        // std::cout << "select + path ;" << Config::select_path << std::endl;
    }

    int getPathCount(std::string dir)
    {
        dir = root_path + dir;
        std::vector<std::string> current_files = get_subdirectories(dir);
        int number = 0;
        for (size_t i = 0; i < current_files.size(); ++i)
        {
            number = std::max(number, std::stoi(current_files[i]));
        }
        return number + 1;
    }

    std::string getSavePath(int type)
    {
        std::string base_path = Config::select_path;
        std::string full_path;

        if (type == 1)
        {
            full_path = base_path + Config::lidar_256_path + "/Sequences";
        }
        else if (type == 2)
        {
            full_path = base_path + Config::lidar_64_path + "/Sequences";
        }
        else if (type == 3)
        {
            full_path = base_path + "/Sequences";
        }
        else
        {
            // 处理无效类型
            full_path = base_path;
        }

        Config::save_type = type;
        return full_path;
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

    bool is_usb_inserted()
    {
        const std::string usb_mount_path = "/mnt/usb";

        struct stat st;
        // 判断 /mnt/usb 是否存在并且是目录
        if (stat(usb_mount_path.c_str(), &st) != 0 || !S_ISDIR(st.st_mode))
        {
            return false; // 目录不存在或不是目录
        }

        // 判断该目录是否包含内容（非 . 和 ..）
        DIR *dir = opendir(usb_mount_path.c_str());
        if (!dir)
        {
            return false;
        }

        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string name = entry->d_name;
            if (name != "." && name != "..")
            {
                closedir(dir);
                return true; // 有内容，认为 U 盘存在
            }
        }

        closedir(dir);
        return false; // 目录为空，可能未插入 U 盘
    }

    std::string create_usb_session_folder()
    {
        const std::string usb_mount_path = "/mnt/usb";
        return usb_mount_path;
    }

    std::string get_parent_path(const std::string &path)
    {
        size_t pos = path.find_last_of("/\\");
        if (pos == std::string::npos)
            return ""; // 无法找到分隔符
        return path.substr(0, pos);
    }

    bool is_directory(const std::string &path)
    {
        struct stat st;
        return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
    }

    bool create_directory(const std::string &path)
    {
        return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
    }

    bool move_item(const std::string &src, const std::string &dst)
    {
        if(is_directory(dst)){
            std::string cmd1 = "rm -r -f " + dst;
            int ret = system(cmd1.c_str());
        }
        std::string cmd = "mv \"" + src + "\" \"" + dst + "\"";
        int ret = system(cmd.c_str());
        if (ret != 0)
        {
            std::cerr << "Failed to move item from " << src << " to " << dst << std::endl;
            return false;
        }
        return true;
    }

    std::string move_folder_contents(std::string &src_folder, const std::string &dst_folder)
    {
        if (!is_directory(src_folder))
        {
            return "源目录不存在: " + src_folder;
        }

        if (!is_directory(dst_folder))
        {
            return "目标目录不存在: " + src_folder;
        }

        DIR *dir = opendir(src_folder.c_str());
        if (!dir)
        {
            return "无法打开源目录: " + src_folder + " 请先录制数据";
        }
        src_folder = get_parent_path(src_folder);
        src_folder = get_parent_path(src_folder);
        if (!move_item(src_folder, dst_folder))
        {
            return "移动失败: " + src_folder + " -> " + dst_folder;
        }

        closedir(dir);
        return "";
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
