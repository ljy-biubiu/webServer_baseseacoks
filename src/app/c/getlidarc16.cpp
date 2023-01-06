#include "getlidarc16.h"
#include <QDebug>
#include <thread>

GetlidarC16::GetlidarC16(const int port, const std::string ip) {

    udp_init(&sock1, &clientAddr1, port, ip);

    lidarDatas = new std::queue<LidarDataCHXXX>;
    std::thread dowork_thread([this]() { run(); });
    dowork_thread.detach();
}


void GetlidarC16::udp_init(int* sock_num, sockaddr_in* sockaddr, int server_port, const std::string ip) {
    UnInitialize(sock_num);
    sockaddr->sin_family = AF_INET;
    sockaddr->sin_addr.s_addr = inet_addr(ip.c_str());
    sockaddr->sin_port = htons(server_port);
    *sock_num = socket(AF_INET, SOCK_DGRAM, 0);
    if (bind(*sock_num, (struct sockaddr*) sockaddr, sizeof(*sockaddr)) < 0) {
        std::cout << "server bind error!" << std::endl;
    }
}

void GetlidarC16::UnInitialize(int* sock_num) {
    shutdown(*sock_num, SHUT_RDWR);
}

LidarDataCHXXX GetlidarC16::getPointData() {
    LidarDataCHXXX data;
    if (lidarDatas->size() == 0) {
        return data;
    }
    mutex_queue_.lock();
    data = lidarDatas->front();
    lidarDatas->pop();
    mutex_queue_.unlock();
    // LidarDataCHXXX data;
    // LidarDataCHXXX data_reset;
    // data = glaboldata;
    // glaboldata =data_reset;
    return data;
}


void GetlidarC16::run() {

    unsigned int size = sizeof(sockaddr_in);
    clientAddr1 = addr1;
    char recvBuf[1206] = {0};
    int recvLen;
    LidarDataCHXXX lidardata;
    lidardata.angle.resize(128);
    lidardata.distance.resize(128);
    lidardata.intensity.resize(128);
    std::vector<std::vector<float>> angle;
    std::vector<std::vector<float>> distance;
    std::vector<std::vector<int>> intensity;
    angle.clear();
    angle.resize(128);
    distance.clear();
    distance.resize(128);
    intensity.clear();
    intensity.resize(128);


    

    sleep(1);
    while (true) {
        recvLen = recvfrom(sock1, recvBuf, sizeof(recvBuf), 0, (struct sockaddr*) &clientAddr1, &size);
        if (recvLen > 0) {
            std::vector<unsigned char> data;
            for (int i = 0; i < 1200; i++) {
                data.push_back(recvBuf[i]);
            }
            for (int i = 0; i < 1197; i += 7) {

                if (data[i] == 0xff && data[i + 1] == 0xaa && data[i + 2] == 0xbb && data[i + 3] == 0xcc && data[i + 4] == 0xdd && data[i + 5] == 0xee && data[i + 6] == 0x11) {
                    lidardata.distance.clear();
                    lidardata.intensity.clear();
                    lidardata.angle.clear();
                    if (lidardata.angle.empty()) {
                        lidardata.distance = distance;
                        lidardata.intensity = intensity;
                        lidardata.angle = angle;

                        mutex_queue_.lock();

                        // static int i{0};
                        // lidardata.mark = ++i; 
                        
                        
                        lidarDatas->push(lidardata);
                        // glaboldata = lidardata;

                        if (lidarDatas->size() > 5) {
                            //std::cout << "数据缓存大于" << lidarDatas->size() << std::endl;
                            lidarDatas->pop();
                        }
                        mutex_queue_.unlock();
                    }
                    angle.clear();
                    angle.resize(128);
                    distance.clear();
                    distance.resize(128);
                    intensity.clear();
                    intensity.resize(128);
                } else {
                    int linenum = data[i];
                    angle[linenum].push_back((data[i + 1] * 256 + data[i + 2]) / 100.f);
                    distance[linenum].push_back((data[3 + i] * 256 + data[4 + i] + data[5 + i] / 256.f) / 100.f);
                    intensity[linenum].push_back(data[6 + i]);
                }
            }
        }
    }
}
