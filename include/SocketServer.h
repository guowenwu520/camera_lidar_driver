// socket_server.h
#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include <iostream>
#include <string>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstring>
#include "Benewakelidar.cpp"
class BenewakeLidarManager;
class FileManager;

class SocketServer
{
public:
    SocketServer(int port);
    ~SocketServer();

    void start();

private:
    int server_fd, client_fd;
    int port;
    struct sockaddr_in server_addr, client_addr;
    void handle_client(int client_socket, BenewakeLidarManager &benewakeLidarManager, FileManager &fileManager);
    std::string dealBeneWakeLidar(BenewakeLidarManager &benewakeLidarManager, bool isOFF);
    std::string get_init_info( FileManager &fileManager,BenewakeLidarManager &benewakeLidarManager);
    std::string process_command(const std::string &command,
                                BenewakeLidarManager &benewakeLidarManager,
                                FileManager &fileManager);
};

#endif
