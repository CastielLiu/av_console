
#ifndef av_console_QNODE_HPP_
#define av_console_QNODE_HPP_
#include "structs.hpp"
#include <functional>

#ifndef Q_MOC_RUN    //避免Qt的Moc工具对Boost的代码进行Moc
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <driverless_common/DoDriverlessTaskAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <nav_msgs/Odometry.h>
#include <driverless_common/SystemState.h>
#include <driverless_common/common.h>
#endif

#include <unordered_map>
#include "utils.hpp"
#include <thread>
#include <string>
#include <QThread>
#include <QDebug>
#include <QMetaType>
#include <QStringListModel>

namespace av_console {

class Sensor
{
public:
    bool status = false;
    double last_update_time = 0;
    std::string name;
    std::string topic;
    int id;
};

class QNode : public QThread {
    Q_OBJECT
public:
    typedef actionlib::SimpleActionClient<driverless_common::DoDriverlessTaskAction> DoDriverlessTaskClient;

	QNode(int argc, char** argv );
	virtual ~QNode();
    bool init(int try_num, const std::string &master_url="", const std::string &host_url="");
    bool waitForDriverlessServer(float timeout);
	void run();

    enum LogLevel {Debug, Info,Warn,Error,Fatal};
    enum TaskState
    {
        Driverless_Offline,    //自动驾驶离线
        Driverless_Idle,       //自动驾驶空闲
        Driverless_Starting,   //自动驾驶任务启动中
        Driverless_Running,    //自动驾驶任务运行中
        Driverless_Complete,   //自动驾驶任务完成
        Driverless_Failed,     //自动驾驶失败
    };
    void changeTaskState(int state) {task_state_ = state;}
	QStringListModel* loggingModel() { return &logging_model; }
    void log(const std::string &msg);
    void stampedLog( const LogLevel &level, const std::string &msg);
    bool initialed(){return is_init&&ros::master::check() ;}
    bool serverConnected();
    void requestDriverlessTask(const driverless_common::DoDriverlessTaskGoal& goal);
    void cancleAllGoals();
    bool addSensor(int id, const std::string& name, const std::string& topic)
    {
        sensors[id].id = id; sensors[id].name = name;
        sensors[id].topic = topic;
    }

private:
    void gps_callback(const nav_msgs::Odometry::ConstPtr& odom, int sensor_id);
    void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& , int sensor_id);
    void image_callback(const sensor_msgs::Image::ConstPtr& , int sensor_id);
    void diagnostic_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg);
    void sensorStatusTimer_callback(const ros::TimerEvent& );
    void driverlessStatus_callback(const driverless_common::SystemState::ConstPtr& msg);

    void taskFeedbackCallback(const driverless_common::DoDriverlessTaskFeedbackConstPtr& fd);
    void taskDoneCallback(const actionlib::SimpleClientGoalState& state,
                                 const driverless_common::DoDriverlessTaskResultConstPtr& res);
    void taskActivedCallback();

Q_SIGNALS:
  void loggingUpdated();
  void sensorStatusChanged(int sensorId,bool status);
  void taskStateChanged(int state, const QString& info="");
  void rosmasterOffline();
  void showDiagnosticMsg(const QString& device, int level,const QString& msg);
  void driverlessStatusChanged(const driverless_common::SystemState state);

private:
	int init_argc;
	char** init_argv;

    QStringListModel logging_model;
	bool is_init;

    std::unordered_map<int, Sensor> sensors;
    const float sensor_detect_interval = 0.5;

    std::vector<ros::Subscriber> subs;
    ros::Subscriber diagnostic_sub;
    ros::Subscriber status_sub;
    ros::Timer      sensorStatus_timer;

    DoDriverlessTaskClient* ac_;
    bool as_online_;
    int task_state_;
};

}  // namespace av_console

#endif /* av_console_QNODE_HPP_ */
