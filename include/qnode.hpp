
#ifndef av_console_QNODE_HPP_
#define av_console_QNODE_HPP_
#include "structs.hpp"

#ifndef Q_MOC_RUN    //避免Qt的Moc工具对Boost的代码进行Moc
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <driverless/DoDriverlessTaskAction.h>
#endif

#include <thread>
#include <string>
#include <QThread>
#include <QDebug>
#include <QStringListModel>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace av_console {

class QNode : public QThread {
    Q_OBJECT
public:
    typedef actionlib::SimpleActionClient<driverless::DoDriverlessTaskAction> DoDriverlessTaskClient;

	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
    void roscoreThread() {system("roscore");}
    void initActionlibClient();
	void run();

    enum LogLevel {Debug, Info,Warn,Error,Fatal};

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
	bool initialed(){return is_init;}

private:
    void startSensorCheck();
    void gpsFix_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix);
    void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& );
    void diagnostic_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg);
    void sensorStatusTimer_callback(const ros::TimerEvent& );

Q_SIGNALS:
  void loggingUpdated();
  void sensorStatusChanged(int sensorId,bool status);

private:
	int init_argc;
	char** init_argv;

    QStringListModel logging_model;
	bool is_init;

    sensor_t gps,rtk,camera1,lidar,esr;
    std::vector<sensor_t*> sensors;
    ros::Subscriber gps_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber diagnostic_sub;
    ros::Timer      sensorStatus_timer;

    DoDriverlessTaskClient* ac_;

};

}  // namespace av_console

#endif /* av_console_QNODE_HPP_ */
