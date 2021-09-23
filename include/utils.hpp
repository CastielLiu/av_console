#ifndef UTILS_HPP
#define UTILS_HPP
#include <iostream>
#include <fstream>
#include <cstdio>
#include "structs.hpp"
#include "globalvariables.hpp"
#include <sstream>
#include <thread>
#include <QStringList>

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

/*@brief  启动ros nodes节点
 *@param  nodes_name 节点名称
 *@param  is_close 是否关闭(默认为启动)
 */
static bool launchRosNodes(const std::string& nodes_name, bool is_close=false, bool terminal=false)
{
    std::string cmd;
    if(is_close) cmd = g_rosNodesArray[nodes_name].close_cmd;
    else         cmd = g_rosNodesArray[nodes_name].launch_cmd;

    if(cmd.empty()) return false;

    if(terminal)
    {
        cmd = std::string("gnome-terminal -x ") + cmd;
        int pos = cmd.find_last_of('&');
        if(pos != std::string::npos)
            cmd = cmd.substr(0,pos); //删除后台运行标识符

        std::cout << cmd << std::endl;
    }

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

static void displayRosNodesArrayInfo()
{
    for(RosNodesArray::iterator iter=g_rosNodesArray.begin(); iter!=g_rosNodesArray.end(); ++iter)
    {
        std::string name = iter->first;
        RosNodes& nodes = iter->second;
        std::cout << "nodes name: " << name << "\t" << "use_button:" << nodes.use_button << std::endl;
        std::cout << "launch cmd: " << nodes.launch_cmd << std::endl;
        std::cout << "close  cmd: " << nodes.close_cmd << std::endl;

        for(auto it=nodes.topics.begin(); it != nodes.topics.end(); ++it)
            std::cout << "topic[" << it->first << "]: " << it->second << std::endl;
        std::cout << "--------------------\r\n";
    }
}

// 提取不带后缀的文件名
static QString extractNoneSuffixFileName(const QString& long_name)
{
    QStringList list1 = long_name.split('/');
    QString rel_name = list1.back();
    QStringList list2 = rel_name.split('.');
    return list2.front();
}

static QString extractLastLevelDirName(const QString& long_name)
{
    QStringList list1 = long_name.split('/');
    return list1.back();
}



}
#endif // UTILS_HPP
