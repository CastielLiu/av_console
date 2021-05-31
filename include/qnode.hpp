
#ifndef av_console_QNODE_HPP_
#define av_console_QNODE_HPP_
#include "structs.hpp"
#include <functional>

#ifndef Q_MOC_RUN    //避免Qt的Moc工具对Boost的代码进行Moc
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <driverless/DoDriverlessTaskAction.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <driverless/State.h>
#endif

#include <thread>
#include <string>
#include <QThread>
#include <QDebug>
#include <QStringListModel>

namespace av_console {

class QNode : public QThread {
    Q_OBJECT
public:
    typedef actionlib::SimpleActionClient<driverless::DoDriverlessTaskAction> DoDriverlessTaskClient;

	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
    void roscoreThread() {system("roscore&");}
	void run();

    enum LogLevel {Debug, Info,Warn,Error,Fatal};
    enum TaskState
    {
        Idle,
        Driverless_Starting,   //自动驾驶任务启动中
        Driverless_Running,    //自动驾驶任务运行中
        Driverless_Complete,   //自动驾驶任务完成
    };

	QStringListModel* loggingModel() { return &logging_model; }
    void log(const std::string &msg);
    void stampedLog( const LogLevel &level, const std::string &msg);
	bool initialed(){return is_init;}
    bool serverConnected();
    void requestDriverlessTask(const driverless::DoDriverlessTaskGoal& goal);
    void cancleAllGoals();
    int  getSensorCnt()const {return sensors.size();}

private:
    void driverlessState_callback(const driverless::State::ConstPtr& msg);
    void gps_callback(const nav_msgs::Odometry::ConstPtr& gps_msg);
    void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& );
    void livox_callback(const sensor_msgs::PointCloud2::ConstPtr& );
    void location_callback(const nav_msgs::Odometry::ConstPtr& location_msg);
    //void diagnostic_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg);
    void sensorStatusTimer_callback(const ros::TimerEvent& );

    void taskFeedbackCallback(const driverless::DoDriverlessTaskFeedbackConstPtr& fd);
    void taskDoneCallback(const actionlib::SimpleClientGoalState& state,
                                 const driverless::DoDriverlessTaskResultConstPtr& res);
    void taskActivedCallback();

Q_SIGNALS:
  void loggingUpdated();
  void sensorStatusChanged(int sensorId,bool status);
  void taskStateChanged(int state);
  void rosmasterOffline();
  void driverlessStatusChanged(float speed,float steerAngle,float latErr);

private:
	int init_argc;
	char** init_argv;

    QStringListModel logging_model;
	bool is_init;

    sensor_t gps,camera1,lidar,livox,location;
    std::vector<sensor_t*> sensors;
    ros::Subscriber driverless_state_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber livox_sub;
    ros::Subscriber location_sub;
    ros::Timer      sensorStatus_timer;

    DoDriverlessTaskClient* ac_;
    bool as_online_;
    int task_state_;
};

}  // namespace av_console

#endif /* av_console_QNODE_HPP_ */
