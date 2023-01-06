#ifndef GETLIDARC16_H
#define GETLIDARC16_H


#include <iostream>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <queue>
#include <mutex>
#include <unistd.h> 


struct LidarDataCHXXX
{
    // int mark;
    std::vector<std::vector<float> >angle;
    std::vector<std::vector<float> >distance;
    std::vector<std::vector<int> >intensity;
};

class GetlidarC16 {
public:
    GetlidarC16(const int port, const std::string ip);

    int count = 0;
    bool contrl = false;

    void udp_init(int* sock_num, sockaddr_in* sockaddr, int server_port, const std::string ip);
    void UnInitialize(int* sock_num);
    int m_sock;
    void run();

    LidarDataCHXXX getPointData();

    int sock1;
    struct sockaddr_in addr1;
    struct sockaddr_in clientAddr1;

    std::queue<LidarDataCHXXX>* lidarDatas;
    std::mutex mutex_queue_;

    LidarDataCHXXX glaboldata;

    void SendData(LidarDataCHXXX lidardata);
};


#endif // GETLIDARC16_H
