#ifndef UTILS_HPP
#define UTILS_HPP
#include <iostream>
#include <fstream>
#include <cstdio>
#include "structs.hpp"

namespace av_console {

//启动GPS节点
static void launchGpsNode()
{

}
//启动IMU节点
static void launchImuNode()
{

}
//启动汽车状态节点
static void launchVehicleStateNode()
{

}

static bool loadPathPoints(std::string file_path,PoseArray& path)
{
    std::ifstream in_file(file_path.c_str());
    if(!in_file.is_open())
    {
        printf("open %s failed",file_path.c_str());
        return false;
    }
    Pose pose;
    std::string line;
    path.poses.clear();
    while(in_file.good())
    {
        getline(in_file,line);
        std::stringstream ss(line);
        if(line.length() == 0)
            break;
        ss >> pose.x >> pose.y >> pose.yaw;
        std::cout << pose.x << "\t" <<  pose.y << "\t" << pose.yaw << std::endl;
        path.poses.push_back(pose);
    }
    return true;
}

}
#endif // UTILS_HPP
