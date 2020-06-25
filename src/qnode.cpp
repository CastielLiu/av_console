/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace av_console {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),
	is_init(false)
{
    sensors.resize(5);
    sensors[Sensor_Gps] = &gps;
    sensors[Sensor_Rtk] = &rtk;
    sensors[Sensor_Camera1] = &camera1;
    sensors[Sensor_Lidar] = &lidar;
    sensors[Sensor_Esr] = &esr;
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"av_console");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	start();
	is_init = true;
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"av_console");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	start();
	is_init = true;
	return true;
}

void QNode::run()
{
    startSensorCheck();
    ros::spin();

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/*===================传感器状态监测相关函数================*/
/*官方驱动则使用消息回调检测，用户自定义驱动则使用故障诊断消息检测*/
void QNode::startSensorCheck()
{
    ros::NodeHandle nh;
    gps_sub = nh.subscribe("/gps_fix",1,&QNode::gpsFix_callback,this);
    lidar_sub =  nh.subscribe("/pandar_points",1,&QNode::lidar_callback,this);
    diagnostic_sub = nh.subscribe("/sensors/diagnostic",10,&QNode::diagnostic_callback,this);
    sensorStatus_timer = nh.createTimer(ros::Duration(1), &QNode::sensorStatusTimer_callback,this);
}

void QNode::gpsFix_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix)
{
    //qDebug() << "gpsFix_callback ";
    double time = ros::Time::now().toSec();
    gps.last_update_time = time;

    if(gps_fix->status.status > 8)
        rtk.last_update_time = time;
}

void QNode::lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& )
{
    //qDebug() << "lidar_callback ";
    static int i = 0;
    if((++i)%5 == 0) //降低更新时间覆盖频率
        lidar.last_update_time = ros::Time::now().toSec();
}

void QNode::diagnostic_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg)
{
    if(msg->hardware_id=="esr_radar")
        esr.last_update_time = ros::Time::now().toSec();
    else if(msg->hardware_id=="camera")
        camera1.last_update_time = ros::Time::now().toSec();
}
void QNode::sensorStatusTimer_callback(const ros::TimerEvent& )
{
    static float tolerateInterval = 1.0;
    double now = ros::Time::now().toSec();
    for(size_t i=0; i<sensors.size(); ++i)
    {
        float interval = now - sensors[i]->last_update_time;
        if(sensors[i]->status==true && interval > tolerateInterval)
        {
            sensors[i]->status = false;
            Q_EMIT sensorStatusChanged(i,false);
            //qDebug() << "sensorStatusTimer_callback " << i << "\t" << sensors[i]->status;
        }
        else if(sensors[i]->status==false && interval < tolerateInterval)
        {
            sensors[i]->status = true;
            Q_EMIT sensorStatusChanged(i,true);
            //qDebug() << "sensorStatusTimer_callback " << i << "\t" << sensors[i]->status;
        }
    }
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace av_console
