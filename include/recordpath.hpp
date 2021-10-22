#ifndef RECORDPATH_H
#define RECORDPATH_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <driverless_common/common.h>
#endif

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QDir>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <cmath>
#include <QTimer>

#include <vector>
#include <QStringListModel>
#include <fstream>
#include <sstream>
#include "globalvariables.hpp"
#include <mutex>
#include <memory>

typedef GpsPoint PathPoint;

class RecordPath : public QObject
{
 Q_OBJECT
private:
    float dis2Points(const PathPoint& p1, const PathPoint&p2,bool isSqrt);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool calPathCurvature(std::vector<PathPoint>& points);

    std::string file_path_;
    std::string file_name_;

    PathPoint last_point_;
    float sample_distance_;
    ros::Subscriber sub_location_odom_;
    std::string odom_topic_;

    PathPoint current_point_;
    std::vector<PathPoint> path_points_;
    std::string points_title_;
    QStringListModel logging_model;

    QTimer  wait_topic_timer_;
    bool recording_;

    std::mutex mutex_;
    float current_road_left_width_;
    float current_road_right_width_;
    std::vector<TurnRange> turn_ranges_;
    std::vector<ParkingPoint> park_points_;
    std::vector<TrafficLightPoint> traffic_light_points_;
    std::vector<SpeedRange> speed_ranges_;

Q_SIGNALS:
    void loggingUpdated();

public Q_SLOTS:
    void waitTopicTimeout();

public:
    RecordPath();
    virtual ~RecordPath();
    bool start();
    void stop();
    std::string odomTopic(){return odom_topic_;}
    QStringListModel* loggingModel() { return &logging_model; }
    void abandon();
    bool save(const std::string& dir);
    void log( const std::string &level, const std::string &msg);
    void setRoadWidth(float left, float right);
    void setTurnRange(const std::string &type, size_t startIdx, size_t endIdx);
    void setParkPoint(size_t duration);
    void setTrafficLightPoint();
    void setMaxSpeed(float speed, bool is_start);
    size_t getPointsSize() {std::lock_guard<std::mutex> lck(mutex_); return path_points_.size();}

private:
    bool savePathPoints(const std::string& );
    bool saveExtendPathInfo(const std::string& file_name);
};

/*@brief 获取两点间的距离
 *@param point1 终点
 *@param point2 起点
 */
static float getDistance(const PathPoint& point1, const PathPoint& point2)
{
    float x = point1.x - point2.x;
    float y = point1.y - point2.y;
    return sqrt(x * x + y * y);
}


#endif
