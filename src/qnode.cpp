
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qnode.hpp"

namespace av_console {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),
    is_init(false),
    as_online_(false),
    task_state_(Driverless_Idle),
    ac_(nullptr)
{
    sensors.resize(Sensor::TotalCount);
}

QNode::~QNode()
{
    if(ac_ != nullptr)
    {
        delete ac_;
        ac_ = nullptr;
    }
    if(ros::isStarted())
    {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init(int try_num, const std::string &master_url, const std::string &host_url)
{
    if(!master_url.empty() && !host_url.empty())
    {
        std::map<std::string,std::string> remappings;
        remappings["__master"] = master_url;
        remappings["__hostname"] = host_url;
        ros::init(remappings,"av_console");
    }
    else
        ros::init(init_argc,init_argv,"av_console");

    while ( ! ros::master::check() && try_num>0)
    {
       try_num --;
       execute_process_detached("roscore");
       QThread::msleep(1000);
    }

    if(try_num <= 0)  return false;

    //subscribe sensor msg to listen its status.
    ros::NodeHandle nh;
    gps_sub = nh.subscribe(g_rosNodesArray["gps"].topics["odom"],1,&QNode::gpsOdom_callback,this);
    lidar_sub =  nh.subscribe(g_rosNodesArray["lidar"].topics["points"],1,&QNode::lidar_callback,this);
    diagnostic_sub = nh.subscribe("/driverless/diagnostic",10,&QNode::diagnostic_callback,this);
    sensorStatus_timer = nh.createTimer(ros::Duration(1), &QNode::sensorStatusTimer_callback,this);

    if(ac_ != nullptr)
    {
        delete ac_;
        ac_ = nullptr;
    }

    ac_ = new DoDriverlessTaskClient("/do_driverless_task", true); // true -> don't need ros::spin()
    is_init = true;

    QThread::msleep(500); //wait for stable networking
	return true;
}

bool QNode::waitForDriverlessServer(float timeout)
{
    float time = 0;
    if(this->serverConnected())
        return true;

    launchRosNodes("driverless");

    while(!this->serverConnected())
    {
        QThread::msleep(300);
        time += 0.3;
        if(time > timeout)
            return false;
    }
    return true;
}

/*QThread 线程函数，start后开始执行*/
void QNode::run()
{
    ros::AsyncSpinner spinner(5);
    spinner.start(); //非阻塞

    while(ros::ok() && ros::master::check())
    {
        if(!ac_->isServerConnected())
        {
            Q_EMIT taskStateChanged(Driverless_Offline);
        }

        ros::Duration(1.0).sleep();
    }
    if(!ros::master::check())
    {
        Q_EMIT rosmasterOffline();
        is_init = false;
    }
}

bool QNode::serverConnected()
{
    return ac_->isServerConnected();

}

void QNode::cancleAllGoals()
{
    std::cout << "cancleAllGoals" << std::endl;
    ac_->cancelAllGoals();
}

void QNode::requestDriverlessTask(const driverless_actions::DoDriverlessTaskGoal& goal)
{
    ac_->sendGoal(goal, boost::bind(&QNode::taskDoneCallback,this,_1,_2),
                        boost::bind(&QNode::taskActivedCallback,this),
                        boost::bind(&QNode::taskFeedbackCallback,this,_1));
}

void QNode::taskFeedbackCallback(const driverless_actions::DoDriverlessTaskFeedbackConstPtr& fd)
{
    qDebug() << "taskFeedbackCallback ";
}

void QNode::taskDoneCallback(const actionlib::SimpleClientGoalState& state,
                             const driverless_actions::DoDriverlessTaskResultConstPtr& res)
{
    std::cout << "taskDoneCallback " << state.toString().c_str() << std::endl;

    Q_EMIT taskStateChanged(Driverless_Complete, QString(state.getText().c_str()));
}

void QNode::taskActivedCallback()
{
    qDebug() << "taskActivedCallback ";
    Q_EMIT taskStateChanged(Driverless_Running);
}

/*===================传感器状态监测相关函数================*/
/*官方驱动则使用消息回调检测，用户自定义驱动则使用故障诊断消息检测*/
void QNode::gpsOdom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    //qDebug() << "gpsOdom_callback ";
    static int i = 0;
    if((++i)%10 != 0) //降低更新时间覆盖频率
        return ;

    double time = ros::Time::now().toSec();

    int location_type = odom->pose.covariance[4];
    int satellite_num = odom->pose.covariance[3];

    if(location_type == 1 || location_type == 2)
        sensors[Sensor::Gps].last_update_time = time;
    if(location_type ==4 || location_type == 5 || location_type==9)
    {
        sensors[Sensor::Gps].last_update_time = time;
        sensors[Sensor::Rtk].last_update_time = time;
    }
}

void QNode::lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& )
{
    //qDebug() << "lidar_callback ";
    static int i = 0;
    if((++i)%5 == 0) //降低更新时间覆盖频率
        sensors[Sensor::Lidar].last_update_time = ros::Time::now().toSec();
}

void QNode::diagnostic_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg)
{
    //ros消息中的字符串需要通过字符串流取出
    std::string hardware_id, message; char level;
    std::stringstream ss1; ss1 << msg->hardware_id; ss1 >> hardware_id;
    std::stringstream ss2; ss2 << msg->level; ss2 >> level;  //std::cout << ss2.str() << std::endl;
    std::stringstream ss3; ss3 << msg->message; ss3 >> message;

    //std::cout << hardware_id << " " <<  level << " " << message << std::endl;

    Q_EMIT showDiagnosticMsg(QString::fromStdString(hardware_id),level,QString::fromStdString(msg->message));
}

//传感器状态更新定时器，若新消息超时，则认为数据丢失
void QNode::sensorStatusTimer_callback(const ros::TimerEvent& )
{
    //qDebug() << "sensorStatusTimer_callback\r\n" ;
    static float tolerateInterval = 1.0;
    double now = ros::Time::now().toSec();
    for(size_t i=0; i<sensors.size(); ++i)
    {
        float interval = now - sensors[i].last_update_time;
        if(sensors[i].status==true && interval > tolerateInterval)
        {
            sensors[i].status = false;
            Q_EMIT sensorStatusChanged(i,false);
            //qDebug() << "sensorStatusTimer_callback " << i << "\t" << sensors[i].status;
        }
        else if(sensors[i].status==false && interval < tolerateInterval)
        {
            sensors[i].status = true;
            Q_EMIT sensorStatusChanged(i,true);
            //qDebug() << "sensorStatusTimer_callback " << i << "\t" << sensors[i].status;
        }
    }
}

void QNode::log(const std::string &msg)
{
    logging_model.insertRows(logging_model.rowCount(),1);

    QVariant new_row(QString(msg.c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar

}

void QNode::stampedLog( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
    double now = ros::Time::now().toSec();

    logging_model_msg << std::fixed << std::setprecision(2);

	switch ( level ) {
        case(Debug) :
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << now << "]: " << msg;
            break;
        case(Info) :
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << now << "]: " << msg;
            break;
        case(Warn) :
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << now << "]: " << msg;
            break;
        case(Error) :
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << now << "]: " << msg;
            break;
        case(Fatal) :
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << now << "]: " << msg;
            break;
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace av_console
