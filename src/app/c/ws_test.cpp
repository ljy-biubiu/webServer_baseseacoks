// Copyright (c) 2013-2017, Matt Godbolt
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// An extraordinarily simple test which presents a web page with some buttons.
// Clicking on the numbered button increments the number, which is visible to
// other connected clients.  WebSockets are used to do this: by the rather
// suspicious means of sending raw JavaScript commands to be executed on other
// clients.

#include "seasocks/PrintfLogger.h"
#include "seasocks/Server.h"
#include "seasocks/StringUtil.h"
#include "seasocks/WebSocket.h"
#include "seasocks/util/Json.h"

#include <cstring>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include <pcl/common/io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h> //loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

#include <jsoncpp/json/json.h>

#include <getlidarc16.h>
#include <thread>
#define PI 3.14159

using namespace seasocks;

class MyHandler : public WebSocket::Handler {
public:
    explicit MyHandler(Server* server)
            : _server(server), _currentValue(0) {
        setValue(1);

        getlidarc16 = new GetlidarC16(2322, "127.0.0.1");
        std::thread dowork_thread([this]() { run(); });
        dowork_thread.detach();
    }

    void onConnect(WebSocket* connection) override {
        _connections.insert(connection);
        connection->send(_currentSetValue.c_str());
        std::cout << "Connected: " << connection->getRequestUri()
                  << " : " << formatAddress(connection->getRemoteAddress())
                  << "\nCredentials: " << *(connection->credentials()) << "\n";
    }

    void run() {
        while (1) {
            LidarDataCHXXX lidarData;
            usleep(50 * 1000);
            lidarData = getlidarc16->getPointData();
            if (lidarData.angle.size() != 0) {
                pointcloud_data = makeExecString("pointcloud",
                                                 CalculateCoordinates(lidarData));
                std::cout<<"t||||||||||||||||||||||||||"<<std::endl;
                std::cout<<"t||||||||||||||||||||||||||"<<std::endl;
                std::cout<<"t||||||||||||||||||||||||||"<<std::endl;
                std::cout<<pointcloud_data<<std::endl;
                mutex_.lock();
                for (auto c : _connections) {
                    c->send(pointcloud_data.c_str());
                }
                mutex_.unlock();
            }
        }
    }

    //     mutex_.lock();
    //  for (auto c : _connections) {
    //     c->send(pointcloud_data.c_str());
    // }
    // mutex_.unlock();

    void sendPointDataToWeb() {
    }

    std::string CalculateCoordinates(LidarDataCHXXX lidardata) {

        // std::cout<<"mark:"<<lidardata.mark<<std::endl;


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tCloud(new pcl::PointCloud<pcl::PointXYZRGB>);


        float sinTheta1[128]; //ÊúÖ±œÇ
        float sinTheta2[128]; //ÊúÖ±œÇ

        float cos_x_angle = cos(XAngle * PI / 180);
        float sin_x_angle = sin(XAngle * PI / 180);
        float cos_y_angle = cos(YAngle * PI / 180);
        float sin_y_angle = sin(YAngle * PI / 180);
        float cos_z_angle = cos(0 * PI / 180);
        float sin_z_angle = sin(0 * PI / 180);


        for (int i = 0; i < 128; i++) {
            sinTheta1[i] = sin(((i % 4) * (-0.17)) * PI / 180.f);
            sinTheta2[i] = sin(BigAngle[i / 4] * PI / 180.f);
            for (int j = 0; j < (int) lidardata.angle[i].size(); j++) {
                if (lidardata.angle[i][j] - int(lidardata.angle[i][j] / 180) * 180 >= 30 && lidardata.angle[i][j] - int(lidardata.angle[i][j] / 180) * 180 <= 150) {
                    float sinTheta = sinTheta2[i] + 2 * cos((lidardata.angle[i][j] * PI / 180) / 2.0) * sinTheta1[i];
                    float cosTheta = sqrt(1 - sinTheta * sinTheta);
                    float sinAngle = sin(lidardata.angle[i][j] * PI / 180.f);
                    float cosAngle = cos(lidardata.angle[i][j] * PI / 180.f);
                    if (lidardata.distance[i][j] < 0.3) {
                        continue;
                    }
                    pcl::PointXYZRGB PointTemp1;
                    PointTemp1.y = (lidardata.distance[i][j] * cosTheta * sinAngle);
                    PointTemp1.x = (lidardata.distance[i][j] * cosTheta * cosAngle);
                    PointTemp1.z = (lidardata.distance[i][j] * sinTheta);

                    //坐标轴方向转换
                    float transformed_x = PointTemp1.x * cos_z_angle * cos_y_angle + PointTemp1.y * (-sin_z_angle * cos_x_angle + cos_z_angle * sin_y_angle * sin_x_angle) + PointTemp1.z * (sin_z_angle * sin_x_angle + cos_z_angle * sin_y_angle * cos_x_angle);
                    float transformed_y = PointTemp1.x * sin_z_angle * cos_y_angle + PointTemp1.y * (cos_z_angle * cos_x_angle + sin_z_angle * sin_y_angle * sin_x_angle) + PointTemp1.z * (sin_z_angle * sin_y_angle * cos_x_angle - cos_z_angle * sin_x_angle);
                    float transformed_z = (-sin_y_angle * PointTemp1.x + cos_y_angle * sin_x_angle * PointTemp1.y + cos_y_angle * cos_x_angle * PointTemp1.z);


                    //点转换
                    PointTemp1.x = transformed_x + Base_X;
                    PointTemp1.y = transformed_y + Base_Y;
                    PointTemp1.z = transformed_z;

                    if (lidardata.intensity[i][j] <= 63) {
                        PointTemp1.r = 0;
                        PointTemp1.g = 254 - 4 * lidardata.intensity[i][j];
                        PointTemp1.b = 255;
                    } else if (lidardata.intensity[i][j] > 63 && lidardata.intensity[i][j] <= 127) {
                        PointTemp1.r = 0;
                        PointTemp1.g = 4 * lidardata.intensity[i][j] - 254;
                        PointTemp1.b = 510 - 4 * lidardata.intensity[i][j];
                    } else if (lidardata.intensity[i][j] > 127 && lidardata.intensity[i][j] <= 191) {
                        PointTemp1.r = 4 * lidardata.intensity[i][j] - 510;
                        PointTemp1.g = 255;
                        PointTemp1.b = 0;
                    } else if (lidardata.intensity[i][j] > 191 && lidardata.intensity[i][j] <= 255) {
                        PointTemp1.r = 255;
                        PointTemp1.g = 1022 - 4 * lidardata.intensity[i][j];
                        PointTemp1.b = 0;
                    }
                    // PointTemp1.a = lidardata.intensity[i][j];
                    tCloud->points.push_back(PointTemp1);
                }
            }
        }


        //子节点
        Json::Value root;
        Json::Value friends;
        Json::FastWriter writer;

        auto caculate2Bit = [](const double &val) {
            char* chCode;
            chCode = new char[20];
            sprintf(chCode, "%.2lf", int(val * 100) * 0.01);
            std::string str(chCode);
            delete[] chCode;
            return str;
            // return std::to_string(int(data * 100) * 0.01);
        };

        for (size_t i = 0; i < tCloud->points.size(); ++i) {
            friends["x"] = Json::Value(caculate2Bit(tCloud->points[i].x));
            friends["y"] = Json::Value(caculate2Bit(tCloud->points[i].y));
            friends["z"] = Json::Value(caculate2Bit(tCloud->points[i].z));

            // friends["z"] = Json::Value(lidardata.mark);
            root.append(friends);
        }

        //std::cout << writer.write(root) << std::endl;
        return writer.write(root);

 

        /////////////
    }


    void onData(WebSocket* connection, const char* data) override {
        if (0 == strcmp("die", data)) {
            _server->terminate();
            return;
        }
        if (0 == strcmp("close", data)) {
            std::cout << "Closing..\n";
            connection->close();
            std::cout << "Closed.\n";
            return;
        }
        if (0 == strcmp("load", data)) {
            std::string pointcloud_data = makeExecString("pointcloud", toWeb_data);
            for (auto c : _connections) {
                c->send(pointcloud_data.c_str());
            }
            return;
        }


        const int value = std::stoi(data) + 1;
        if (value > _currentValue) {
            setValue(value);
            mutex_.lock();
            for (auto c : _connections) {
                c->send(_currentSetValue.c_str());
            }
            mutex_.unlock();
        }
    }

    void onDisconnect(WebSocket* connection) override {
        _connections.erase(connection);
        std::cout << "Disconnected: " << connection->getRequestUri()
                  << " : " << formatAddress(connection->getRemoteAddress()) << "\n";
    }

    void writeFileJson(const pcl::PointCloud<pcl::PointXYZ>::Ptr data) {
        //子节点
        Json::Value root;
        Json::Value friends;
        Json::FastWriter writer;

        for (size_t i = 0; i < data->points.size(); ++i) {
            friends["x"] = Json::Value(data->points[i].x);
            friends["y"] = Json::Value(data->points[i].y);
            friends["z"] = Json::Value(data->points[i].z);
            root.append(friends);
        }
        toWeb_data = writer.write(root);
    }


    void loadPclDatas() {
        //*打开点云文件
        cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);


        // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ljy/xmh_huanwei.pcd", *cloud) == -1) {
        //     PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        // }

        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ljy/0_tiangetest_r.pcd", *cloud) == -1) {
            PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        }
    }

    void init() {
        loadPclDatas();
        writeFileJson(cloud);
    }

private:
    std::set<WebSocket*> _connections;
    Server* _server;
    int _currentValue;
    std::string _currentSetValue;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::string toWeb_data;
    GetlidarC16* getlidarc16;
    std::mutex mutex_;
    std::string pointcloud_data;

    void
    setValue(int value) {
        _currentValue = value;
        _currentSetValue = makeExecString("set", _currentValue);
    }


    float XAngle = 0;
    float YAngle = 0;
    float ZAngle = 0;

    float Base_X = 0;
    float Base_Y = 0;
    float Base_Z = 0;

    double BigAngle[32] = {-17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4.125, -4, -3.125,
                           -3, -2.125, -2, -1.125, -1, -0.125, 0, 0.875, 1, 1.875, 2, 3, 4, 5, 6, 7};
};

bool checkDir() {
    std::string dir = seasocks::getWorkingDir();
    if (!seasocks::endsWith(dir, "seasocks")) {
        std::cerr << "Samples must be run in the main seasocks directory" << std::endl;
        return false;
    }
    return true;
}


int main(int /*argc*/, const char* /*argv*/[]) {

    if (!checkDir())
        return -1;

    auto logger = std::make_shared<PrintfLogger>(Logger::Level::Debug);
    Server server(logger);

    auto handler = std::make_shared<MyHandler>(&server);
    handler->init();
    server.addWebSocketHandler("/ws", handler);
    server.serve("src/ws_test_web", 9090);

    return 0;
}
