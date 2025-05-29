#include "FileManager.h"

std::string FileManager::getRootPath()
{
    return root_path;
}

std::vector<std::string> FileManager::getFileChildPaths()
{
    return get_subdirectories(root_path);
}

void FileManager::setSavePath(std::string name)
{
    Config::select_path = name + "/";
}

int FileManager::getPathCount(std::string dir)
{
    dir = root_path + dir;
    std::vector<std::string> current_files = get_subdirectories(dir);
    int number = 0;
    for (const auto& file : current_files)
    {
        number = std::max(number, std::stoi(file));
    }
    return number + 1;
}

std::string FileManager::getSavePath(int type)
{
    std::string base_path = Config::select_path;
    std::string full_path;

    switch (type)
    {
        case 1: full_path = base_path + Config::lidar_256_path + "/Sequences"; break;
        case 2: full_path = base_path + Config::lidar_64_path + "/Sequences"; break;
        case 3: full_path = base_path + "/Sequences"; break;
        default: full_path = base_path;
    }

    Config::save_type = type;
    return full_path;
}

bool FileManager::createDirectory(const std::string &namefile)
{
    std::string dir = root_path + namefile;
    std::string cmd = "mkdir -p " + dir;
    return system(cmd.c_str()) == 0;
}

bool FileManager::deleteFile(const std::string &filename)
{
    std::string path = root_path + filename;
    std::string cmd = "rm -r -f " + path;
    return system(cmd.c_str()) == 0;
}

bool FileManager::is_usb_inserted()
{
    const std::string usb_mount_path = "/mnt/usb";
    struct stat st;

    if (stat(usb_mount_path.c_str(), &st) != 0 || !S_ISDIR(st.st_mode))
        return false;

    DIR *dir = opendir(usb_mount_path.c_str());
    if (!dir)
        return false;

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string name = entry->d_name;
        if (name != "." && name != "..")
        {
            closedir(dir);
            return true;
        }
    }

    closedir(dir);
    return false;
}

std::string FileManager::create_usb_session_folder()
{
    return "/mnt/usb";
}

std::string FileManager::get_parent_path(const std::string &path)
{
    size_t pos = path.find_last_of("/\\");
    return (pos == std::string::npos) ? "" : path.substr(0, pos);
}

bool FileManager::is_directory(const std::string &path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool FileManager::create_directory(const std::string &path)
{
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

bool FileManager::move_item(const std::string &src, const std::string &dst)
{
    if (is_directory(dst))
    {
        std::string cmd1 = "rm -r -f " + dst;
        system(cmd1.c_str());
    }

    std::string cmd = "mv \"" + src + "\" \"" + dst + "\"";
    return system(cmd.c_str()) == 0;
}

std::string FileManager::move_folder_contents(std::string &src_folder, const std::string &dst_folder)
{
    if (!is_directory(src_folder))
        return "源目录不存在: " + src_folder;

    if (!is_directory(dst_folder))
        return "目标目录不存在: " + src_folder;

    DIR *dir = opendir(src_folder.c_str());
    if (!dir)
        return "无法打开源目录: " + src_folder + " 请先录制数据";

    src_folder = get_parent_path(get_parent_path(src_folder));
    if (!move_item(src_folder, dst_folder))
        return "移动失败: " + src_folder + " -> " + dst_folder;

    closedir(dir);
    return "";
}

void FileManager::savePointCloudAsKITTI(const benewake::BwPointCloud::Ptr &cloud, int number, std::string dir, int frameNum)
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

std::vector<std::string> FileManager::get_subdirectories(const std::string &path)
{
    std::vector<std::string> folders;
    DIR *dir = opendir(path.c_str());
    if (!dir)
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
            folders.push_back(name);
    }

    closedir(dir);
    return folders;
}
