#ifndef STRUCTS_HPP
#define STRUCTS_HPP
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <QString>
#include <unordered_map>

namespace av_console {

typedef struct _Pose
{
    double x,y,yaw;
}Pose;

typedef struct _PoseArray
{
    std::vector<Pose> poses;
}PoseArray;

//
typedef struct _RosNodes
{
    std::string name;
    std::string launch_cmd;
    std::string close_cmd;
    bool use_button;  //是否使用按钮，true: 在UI界面自动生成按钮
    std::string show_status; //是否显示状态，非空: 在UI界面自动生成状态显示按钮，并作为显示名
    uint8_t id;       //节点Id, 根据载入顺序

    //topic_name, topic_value
    std::unordered_map<std::string, std::string> topics;
}RosNodes;

//rosNode_name, rosNodes
typedef std::unordered_map<std::string, RosNodes> RosNodesArray;

class Logg
{
public:
    Logg(){}
    ~Logg()
    {
        if(log_f.is_open())
            log_f.close();
    }
    bool init(const QString& log_file)
    {
        log_f.open(log_file.toStdString().c_str());
        if(!log_f.is_open())
        {
            std::cout << "Open log file: " << log_file.toStdString() << " failed!" << std::endl;
            return false;
        }
        return true;
    }

    // 检验log文件是否已打开
    bool check()
    {
        if(!log_f.is_open())
        {
            std::cout << "Logg is not ready!" << std::endl;
            return false;
        }
        return true;
    }

    Logg& operator<<(const QString& qstr)
    {
        if(!check()) return *this;
        log_f << qstr.toStdString();
        log_f.flush();
        return *this;
    }

    template <class T>
    Logg& operator<<(const T& what)
    {
        if(!check()) return *this;
        log_f << what;
        log_f.flush();
        return *this;
    }


private:
    std::ofstream log_f;
};

}

#endif // STRUCTS_HPP
