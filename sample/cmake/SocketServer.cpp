// socket_server.cpp
#include "SocketServer.h"
#include "File_uitl.cpp"
#include <ctime>
#include <unordered_map>

SocketServer::SocketServer(int port) : port(port)
{
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 1) < 0)
    {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }

    std::cout << "SocketServer listening on port " << port << std::endl;
}

SocketServer::~SocketServer()
{
    close(server_fd);
}
std::string SocketServer::get_init_info( FileManager &fileManager,BenewakeLidarManager &benewakeLidarManager)
{
    std::string link_status = "OK";
    std::string is_has_UP = (fileManager.isHasUsb() ? "True" : "False");
    std::string up_path = (fileManager.isHasUsb() ? fileManager.get_usb_mounts()[0] : "");
    bool has_lidar_64 = false;
    bool has_lidar_248 = benewakeLidarManager.hasLidar();
    std::string root_path = fileManager.getRootPath();
    std::vector<std::string> current_files = fileManager.getFileChildPaths();

    // 拼接 current_file 数组
    std::string current_file_str = "[";
    for (size_t i = 0; i < current_files.size(); ++i)
    {
        current_file_str += "\"" + current_files[i] + "\"";
        if (i != current_files.size() - 1)
            current_file_str += ", ";
    }
    current_file_str += "]";

    // 构造完整字符串
    std::string init_info = "{link_status:" + link_status +
                            ",is_has_UP:" + is_has_UP +
                            ",UP_path:" + up_path +
                            ",64_line_ladar:" + (has_lidar_64 ? "True" : "False") +
                            ",248_line_ladar:" + (has_lidar_248 ? "True" : "False") +
                            ",root_path:" + root_path +
                            ",current_file:" + current_file_str +
                            "}";
    return init_info;                        
}

void SocketServer::start()
{
    socklen_t len = sizeof(client_addr);
    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &len);
    if (client_fd < 0)
    {
        perror("Accept failed");
        return;
    }

    std::cout << "Client connected!" << std::endl;
    FileManager fileManager = FileManager();
    BenewakeLidarManager benewakeLidarManager = BenewakeLidarManager(fileManager);
    benewakeLidarManager.initialize();
    std::string init_info = get_init_info(fileManager, benewakeLidarManager);
    send(client_fd, init_info.c_str(), init_info.size(), 0);
    handle_client(client_fd, benewakeLidarManager, fileManager);
    close(client_fd);
}

void SocketServer::handle_client(int client_socket, BenewakeLidarManager &benewakeLidarManager, FileManager &fileManager)
{
    char buffer[1024];
    while (true)
    {
        memset(buffer, 0, sizeof(buffer));
        ssize_t n = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0)
        {
            std::cout << "Client disconnected." << std::endl;
            break;
        }

        std::string received(buffer);
        std::string response = process_command(received, benewakeLidarManager, fileManager);
        send(client_socket, response.c_str(), response.size(), 0);
    }
}

std::string SocketServer::dealBeneWakeLidar(BenewakeLidarManager &benewakeLidarManager, bool isStart)
{
    std::string status = "1", error = "";

    if (benewakeLidarManager.hasLidar())
    {
        if (isStart)
        {
            std::thread task_thread([&benewakeLidarManager]()
                                    { benewakeLidarManager.start(); });
            task_thread.detach(); // 后台运行
        }
        else
        {
            benewakeLidarManager.stop();
        }
    }
    else
    {
        status = "0";
        error = "lidar 不存在";
    }
    std::string return_info = "({status: " + status +
                              ", path: " + benewakeLidarManager.getSave_Path() +
                              ", log: " + "nono" +
                              ", error: " + error +
                              "})";
    return return_info;
}
std::string SocketServer::process_command(const std::string &command,
                                          BenewakeLidarManager &benewakeLidarManager,
                                          FileManager &fileManager)
{
    // 拆分命令名与参数
    auto pos = command.find(':');
    std::string cmd = (pos != std::string::npos) ? command.substr(0, pos) : command;
    std::string arg = (pos != std::string::npos) ? command.substr(pos + 1) : "";

    if (cmd == "64_line_ladar_start")
    {
        return "None\n";
    }
    else if (cmd == "64_line_ladar_end")
    {
        return "None\n";
    }
    else if (cmd == "256_line_ladar_start")
    {
        return dealBeneWakeLidar(benewakeLidarManager, true);
    }
    else if (cmd == "256_line_ladar_end")
    {

        return dealBeneWakeLidar(benewakeLidarManager, false);
    }
    else if (cmd == "line_ladar_start")
    {
        return "Hello Client!\n";
    }
    else if (cmd == "line_ladar_end")
    {
        return "Goodbye Client!\n";
    }
    else if (cmd == "create_path")
    {
        bool success = fileManager.createDirectory(arg);
        std::string return_info = "({status: " + std::string(success ? "1" : "0") +
                                  ", error: " + "" + "})";
        return return_info;
    }
    else if (cmd == "select_path")
    {
        fileManager.setSavePath(arg);
        std::string return_info = "({status: 1, error: })";
        return return_info;
    }
    else if (cmd == "det_path")
    {
        fileManager.deleteFile(arg);
        std::string init_info = get_init_info(fileManager, benewakeLidarManager);
        return init_info;
    }
    else if (cmd == "close")
    {
        Config::stopRun = false;
        return "The program has been closed and the service needs to be restarted";
    }
    else
    {
        return "Unknown command.\n";
    }
}
