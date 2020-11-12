#ifndef STRUCTS_HPP
#define STRUCTS_HPP
#include <vector>

namespace av_console {

typedef struct Sensor
{
    bool status;
    double last_update_time;
    Sensor()
    {
        status = false;
        last_update_time = 0;
    }
} sensor_t;

enum SensorId
{
    Sensor_Gps =   0,
    Sensor_Lidar = 1,
    Sensor_Esr =   2,
    Sensor_Camera1=3,
    Sensor_Rtk    =4,
};

typedef struct _Pose
{
    double x,y,yaw;
}Pose;

typedef struct _PoseArray
{
    std::vector<Pose> poses;
}PoseArray;

}

#endif // STRUCTS_HPP
