/**
 * @file /include/av_console/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef av_console_QNODE_HPP_
#define av_console_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QDebug>
#include <QStringListModel>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

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

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
	bool initialed(){return is_init;}

    enum SensorId
    {
        Sensor_Gps =   0,
        Sensor_Lidar = 1,
        Sensor_Esr =   2,
        Sensor_Camera1=3,
        Sensor_Rtk    =4,
    };

private:
    void startSensorCheck();
    void gpsFix_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix);
    void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& );
    void diagnostic_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg);
    void sensorStatusTimer_callback(const ros::TimerEvent& );

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
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

};

}  // namespace av_console

#endif /* av_console_QNODE_HPP_ */
