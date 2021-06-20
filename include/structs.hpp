#ifndef STRUCTS_HPP
#define STRUCTS_HPP
#include <string>
#include <vector>
#include <unordered_map>

namespace av_console {

class Sensor
{
public:

    bool status;
    double last_update_time;
    Sensor()
    {
        status = false;
        last_update_time = 0;
    }

    int id;
    //传感器枚举类型，从0开始顺序排列
    enum ID
    {
        Gps    = 0,
        Lidar  = 1,
        Esr    = 2,
        Camera1= 3,
        Rtk    = 4,

        TotalCount = 5, //传感器的个数
    };
    std::vector<std::string> names = {"gps","lidar","esr","camera1","rtk"};
};



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

    //topic_name, topic_value
    std::unordered_map<std::string, std::string> topics;
}RosNodes;

//rosNode_name, rosNodes
typedef std::unordered_map<std::string, RosNodes> RosNodesArray;

}

#endif // STRUCTS_HPP
