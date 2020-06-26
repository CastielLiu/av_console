#include "../include/recordpath.hpp"

RecordPath::RecordPath():
    odom_topic_("/ll2utm_odom"),
    row_num_(0)
{
}

RecordPath::~RecordPath()
{
    path_points_.clear();
}

//若不存在发布者，返回错误，外部自主启动发布者
bool RecordPath::start()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<float>("sample_distance",sample_distance_,0.1);
    sub_gps_ = nh.subscribe(odom_topic_ ,1,&RecordPath::gps_callback,this);

    path_points_.reserve(5000);
    if(sub_gps_.getNumPublishers() == 0)
        return false;
    return true;
}

void RecordPath::stop()
{
    sub_gps_.shutdown();
}

void RecordPath::gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gpsPoint current_point;
  current_point.x = msg->pose.pose.position.x;
  current_point.y = msg->pose.pose.position.y;
  current_point.yaw = msg->pose.covariance[0];

  if(sample_distance_*sample_distance_ <= dis2Points(current_point,last_point_,false))
  {
    path_points_.push_back(current_point);
    //fprintf(fp,"%.3f\t%.3f\t%.4f\n",current_point.x,current_point.y,current_point.yaw);
    last_point_ = current_point;

    std::stringstream msg;
    msg << ++row_num_ << "\t" << std::fixed << std::setprecision(2)
        << current_point.x << "\t"
        << current_point.y << "\t"
        << current_point.yaw*180.0/M_PI;
    log("INFO",msg.str());
  }
}

bool RecordPath::savePathPoints(const std::string& file_name)
{
   std::ofstream out_file;
   out_file.open(file_name.c_str());

   if(!out_file.is_open())
   {
     std::stringstream ss;
     ss << "open " <<  file_name << " failed!";
     log("ERROR",ss.str());
     return false;
   }

   for(size_t i=0; i<path_points_.size(); ++i)
   {
     out_file << std::fixed << std::setprecision(2)
              << path_points_[i].x << "\t" << path_points_[i].y << "\t"
              << path_points_[i].yaw << "\r\n";
   }
   out_file.close();

   std::stringstream ss;
   ss << "path points saved in " << file_name;
   this->log("INFO",ss.str());

   // generate curvature
   FILE * fp =  popen("rospack find av_console", "r");
   char buf[50] ;
   fscanf(fp,"%s",buf);
   pclose(fp);
   QDir cmdDir = QDir::current();//获取当前工作目录
   cmdDir.cd(QString(buf));      //修改目录，仅修改了目录名，未切换
   cmdDir.cd("scripts");
   QDir::setCurrent(cmdDir.absolutePath()); //切换目录

   std::string tool_file = "generate_curvature.py";
   std::string cmd = std::string("python ") + tool_file + " " + file_name ;
   std::cout << "\n=====================================================\n";
   std::cout << "start to generate curvature... "<< std::endl;
   system(cmd.c_str());
   std::cout << "generate curvature complete..." << std::endl;
   std::cout << "=====================================================\n";

   return true;
}

float RecordPath::dis2Points(gpsPoint& p1,gpsPoint&p2,bool isSqrt)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  if(isSqrt)
    return sqrt(dx*dx+dy*dy);
  return dx*dx+dy*dy;
}

void RecordPath::log( const std::string &level, const std::string &msg)
{
  if(logging_model.rowCount() >400)
  {
      logging_model.removeRows(0,200);
  }

  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;

  logging_model_msg << std::fixed << std::setprecision(3)
                    << "[" << level << "]" << "[" << ros::Time::now() << "]: " << msg;

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

#include<tinyxml2.h>
bool RecordPath::generateParkingPointsFile(const std::string& dir)
{
    std::string file = dir + "/parking_points.xml";
    tinyxml2::XMLDocument doc;
    //1.添加声明
    tinyxml2::XMLDeclaration* declaration = doc.NewDeclaration();
    doc.InsertFirstChild(declaration); //在最前插入声明

    //2.创建根节点
    tinyxml2::XMLElement* root = doc.NewElement("ParkingPoints");
    doc.InsertEndChild(root);  //在最后插入根节点

    //3.创建子节点,并插入父节点
    tinyxml2::XMLElement* descriptionEle = doc.NewElement("Description");
    root->InsertEndChild(descriptionEle);

    tinyxml2::XMLElement* idElement = doc.NewElement("id");
    descriptionEle->InsertEndChild(idElement);
    idElement->InsertEndChild(doc.NewText("the sequence of the parking point"));

    tinyxml2::XMLElement* indexElement = doc.NewElement("index");
    descriptionEle->InsertEndChild(indexElement);
    indexElement->InsertEndChild(doc.NewText("the parking point position in global path"));

    tinyxml2::XMLElement* durationElement = doc.NewElement("duration");
    descriptionEle->InsertEndChild(durationElement);
    durationElement->InsertEndChild(doc.NewText("parking time(s), 0 for destination"));

    tinyxml2::XMLElement* addEle = doc.NewElement("add");
    descriptionEle->InsertEndChild(addEle);
    addEle->InsertEndChild(doc.NewText("To add a parking point manually, please follow the format below"));

    tinyxml2::XMLElement* pointElement = doc.NewElement("ParkingPoint");
    root->InsertEndChild(pointElement); //在最后插入节点

    //4.为子节点增加属性
    pointElement->SetAttribute("id", 0);
    pointElement->SetAttribute("index", path_points_.size());
    pointElement->SetAttribute("duration", 0);

    //6.保存xml文件
    doc.SaveFile(file.c_str());
}
