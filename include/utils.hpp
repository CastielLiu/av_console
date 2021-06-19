#ifndef UTILS_HPP
#define UTILS_HPP
#include <iostream>
#include <fstream>
#include <cstdio>
#include "structs.hpp"
#include "globalvariables.hpp"
#include <sstream>
#include <thread>

namespace av_console {

static void execute_process(const std::string& cmd)
{
    system(cmd.c_str());
}

static void execute_process_detached(const std::string& cmd)
{
    std::thread t(execute_process, cmd);
    t.detach();
}

//启动ros nodes节点
static bool launchRosNodes(const std::string& nodes_name)
{
    std::string launch_cmd = g_rosNodesArray[nodes_name].launch_cmd;
    if(launch_cmd.empty())
        return false;

    std::string cmd = /*std::string("gnome-terminal -x ") +*/ launch_cmd;
    //std::cout <<"run: [" << cmd << "]" << std::endl;
    system(cmd.c_str());
    //std::cout <<"run: [" << cmd << "]" << std::endl;
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
