#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <regex>

#include "Globals.h"
#include "benewake_lidar_driver.h"

class FileManager
{
public:
    const std::string root_path = "/root/workspace/save_path/";

    std::string getRootPath();
    std::vector<std::string> getFileChildPaths();
    void setSavePath(std::string name);
    int getPathCount(std::string dir);
    std::string getSavePath(int type);

    bool createDirectory(const std::string &namefile);
    bool deleteFile(const std::string &filename);
    bool is_usb_inserted();
    std::string create_usb_session_folder();
    std::string get_parent_path(const std::string &path);
    bool is_directory(const std::string &path);
    bool create_directory(const std::string &path);
    bool move_item(const std::string &src, const std::string &dst);
    std::string move_folder_contents(std::string &src_folder, const std::string &dst_folder);

    void savePointCloudAsKITTI(const benewake::BwPointCloud::Ptr &cloud, int number, std::string dir, int frameNum);

private:
    static std::vector<std::string> get_subdirectories(const std::string &path);
};

#endif // FILE_MANAGER_H
