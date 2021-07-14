#include "../include/recordpath.hpp"
#include <fstream>

RecordPath::RecordPath():
    current_road_left_width_(0),
    current_road_right_width_(0)
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
    
    odom_topic_ = g_rosNodesArray["location"].topics["odom"];
    if(odom_topic_.empty())
    {
      std::cout << "No odom topic in Node['location']" << std::endl;
      odom_topic_ = "/error/record_path/location/odom_topic";
    }

    sub_location_odom_ = nh.subscribe(odom_topic_ ,1,&RecordPath::odomCallback,this);

    connect(&wait_topic_timer_, SIGNAL(timeout()), this, SLOT(waitTopicTimeout()));

    path_points_.clear();
    path_points_.reserve(5000);
    ros::Duration(0.5).sleep();  //等待订阅器初始化完成，否则即使存在发布者，也有可能被漏检

    if(sub_location_odom_.getNumPublishers() == 0) //检测发布者是否存在
    {
        std::string info = std::string("No ") + odom_topic_ + " publihser!\nPlease check the gps node!";
        this->log("WARN", info);
        return false;
    }

    return true;
}

void RecordPath::waitTopicTimeout()
{
    if(sub_location_odom_.getNumPublishers() == 0)
    {

    }
}

void RecordPath::stop()
{
    sub_location_odom_.shutdown();
}

void RecordPath::setRoadWidth(float left, float right)
{
    std::lock_guard<std::mutex> lck(mutex_);

    current_road_left_width_ = left;
    current_road_right_width_ = right;
}

void RecordPath::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lck(mutex_);

  current_point_.x = msg->pose.pose.position.x;
  current_point_.y = msg->pose.pose.position.y;
  current_point_.yaw = msg->pose.covariance[0];
  current_point_.leftWidth = current_road_left_width_;
  current_point_.rightWidth = current_road_right_width_;

  int gps_state = msg->pose.covariance[4];
  bool gpsValid = (gps_state == 1 || gps_state == 2 || gps_state==4 ||
                   gps_state==5   || gps_state>=9 );

  if(!gpsValid)
  {
    log("ERROR","STOP! No valid location! state: " + std::to_string(msg->pose.covariance[4]) );
    return;
  }

  if(sample_distance_*sample_distance_ <= dis2Points(current_point_,last_point_,false))
  {
    static bool temp_file_opened = false;
    static std::ofstream outTempFile;
    if(!temp_file_opened)
    {
      auto file_name = QDir::current().absolutePath().toStdString() + "/backup_path.txt";
      outTempFile.open(file_name.c_str());
      log("INFO",file_name);
      temp_file_opened = true;
    }

    outTempFile << std::fixed << std::setprecision(3)
            << current_point_.x << "\t"
            << current_point_.y << "\t"
            << current_point_.yaw << std::endl;

    path_points_.push_back(current_point_);
    last_point_ = current_point_;

    //log
    std::stringstream msg;
    msg << path_points_.size() << "  " << std::fixed << std::setprecision(2)
        << current_point_.x << "  "
        << current_point_.y << "  "
        << current_point_.yaw*180.0/M_PI << "  "
        << current_point_.leftWidth << "  "
        << current_point_.rightWidth;
    log("INFO",msg.str());
  }
}

bool RecordPath::calPathCurvature(std::vector<PathPoint>& points)
{
    size_t size = points.size();
    for(int i=0; i<size-1; ++i)
    {
        float delta_theta = normalizeRadAngle(points[i+1].yaw - points[i].yaw); //旋转角
        float arc_length  = getDistance(points[i+1], points[i]); //利用两点间距近似弧长
        if(arc_length == 0)
            points[i].curvature = 0.0; //绝对值偏大
        else
            points[i].curvature = delta_theta/arc_length; //绝对值偏大
            //points[i].curvature = 2*sin(delta_theta/2)/arc_length;

        //车辆转弯半径有限,路径曲率有限，若计算值超出阈值，将其饱和
        if(points[i].curvature>0.3) points[i].curvature = 0.3;
        else if(points[i].curvature<-0.3) points[i].curvature = -0.3;
    }

    //均值滤波
    int n = 10;
    float curvature_n_sum = 0.0;
    for(int i=0; i < size; ++i)
    {
        if(i<n)
            curvature_n_sum+=points[i].curvature;
        else
        {
            points[i-n/2].curvature = curvature_n_sum/n;
            curvature_n_sum += (points[i].curvature - points[i-n].curvature);
            //std::cout << std::fixed << std::setprecision(2) << points[i].curvature << std::endl;
        }
    }
    return true;
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

 #if 1  //cpp gen curvature
   calPathCurvature(path_points_);

   for(size_t i=0; i<path_points_.size(); ++i)
   {
     out_file << std::fixed << std::setprecision(3)
              << path_points_[i].x << "\t" << path_points_[i].y << "\t"
              << path_points_[i].yaw << "\t" << path_points_[i].curvature << "\t"
              << path_points_[i].leftWidth << "\t" << path_points_[i].rightWidth
              << "\r\n";
   }
   out_file.close();
#else //py gen curveature
   for(size_t i=0; i<path_points_.size(); ++i)
   {
     out_file << std::fixed << std::setprecision(2)
              << path_points_[i].x << "\t" << path_points_[i].y << "\t"
              << path_points_[i].yaw << "\r\n";
   }
    out_file.close();

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
#endif

   std::stringstream ss;
   ss << "path points saved in " << file_name;
   this->log("INFO",ss.str());

   return true;
}

float RecordPath::dis2Points(PathPoint& p1,PathPoint&p2,bool isSqrt)
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
                    << "[" << level << "]: " << msg;

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

/*@brief 生成路径信息文件
 *1. parking points
 *2. turn
 *@param file_name 文件
 */
#include<tinyxml2.h>
bool RecordPath::generatePathInfoFile(const std::string& file_name)
{
    tinyxml2::XMLDocument doc;
    //1.添加声明
    tinyxml2::XMLDeclaration* declaration = doc.NewDeclaration();
    doc.InsertFirstChild(declaration); //在最前插入声明

    //2.创建根节点
    tinyxml2::XMLElement* pathInfoNode = doc.NewElement("PathInfo");
    doc.InsertEndChild(pathInfoNode);  //在最后插入根节点

    { //ParkingPoints
    tinyxml2::XMLElement* parkingPointsNode = doc.NewElement("ParkingPoints");
    pathInfoNode->InsertEndChild(parkingPointsNode);

    // 创建Description子节点,并插入父节点
    tinyxml2::XMLElement* discriptionNode = doc.NewElement("Description");
    parkingPointsNode->InsertEndChild(discriptionNode);

    tinyxml2::XMLElement* idElement = doc.NewElement("id");
    discriptionNode->InsertEndChild(idElement);
    idElement->InsertEndChild(doc.NewText("the sequence of the parking point"));

    tinyxml2::XMLElement* indexElement = doc.NewElement("index");
    discriptionNode->InsertEndChild(indexElement);
    indexElement->InsertEndChild(doc.NewText("the parking point position in global path"));

    tinyxml2::XMLElement* durationElement = doc.NewElement("duration");
    discriptionNode->InsertEndChild(durationElement);
    durationElement->InsertEndChild(doc.NewText("parking time(s), -1 for destination"));

    tinyxml2::XMLElement* addEle = doc.NewElement("add");
    discriptionNode->InsertEndChild(addEle);
    addEle->InsertEndChild(doc.NewText("To add a parking point manually, please follow the format below"));

    size_t park_point_id = 0;
    for(size_t i=0; i<park_points_.size(); ++i)
    {
        const ParkingPoint& park_point = park_points_[i];
        tinyxml2::XMLElement* pointElement = doc.NewElement("ParkingPoint");
        parkingPointsNode->InsertEndChild(pointElement); //在最后插入节点
        pointElement->SetAttribute("id", park_point_id++);
        pointElement->SetAttribute("index", park_point.index);
        pointElement->SetAttribute("duration", park_point.parkingDuration);
    }

    //创建ParkingPoint节点
    tinyxml2::XMLElement* pointElement = doc.NewElement("ParkingPoint");
    parkingPointsNode->InsertEndChild(pointElement); //在最后插入节点

    //为节点增加属性,终点停车
    pointElement->SetAttribute("id", park_point_id);
    pointElement->SetAttribute("index", path_points_.size()-1);
    pointElement->SetAttribute("duration", -1);

    }

    {//TurnRanges
    tinyxml2::XMLElement* turnRangesNode = doc.NewElement("TurnRanges");
    pathInfoNode->InsertEndChild(turnRangesNode);

    //创建Description子节点,并插入父节点
    tinyxml2::XMLElement* discriptionNode = doc.NewElement("Description");
    turnRangesNode->InsertEndChild(discriptionNode);

    tinyxml2::XMLElement* typeElement = doc.NewElement("type");
    discriptionNode->InsertEndChild(typeElement);
    typeElement->InsertEndChild(doc.NewText("-1: left turn, 0: none, 1: right turn"));

    tinyxml2::XMLElement* startElement = doc.NewElement("start");
    discriptionNode->InsertEndChild(startElement);
    startElement->InsertEndChild(doc.NewText("the start index of turn"));

    tinyxml2::XMLElement* endElement = doc.NewElement("end");
    discriptionNode->InsertEndChild(endElement);
    endElement->InsertEndChild(doc.NewText("the end index of turn"));

    tinyxml2::XMLElement* addEle = doc.NewElement("add");
    discriptionNode->InsertEndChild(addEle);
    addEle->InsertEndChild(doc.NewText("To add a turn range manually, please follow the format below"));

    //创建TurnRange节点
    tinyxml2::XMLElement* turnRangeNode = doc.NewElement("TurnRange");
    turnRangesNode->InsertEndChild(turnRangeNode);

    //添加属性,示例
    turnRangeNode->SetAttribute("type", 0);
    turnRangeNode->SetAttribute("start", 0);
    turnRangeNode->SetAttribute("end", 15);

    for(const TurnRange& turn_range: turn_ranges_)
    {
        tinyxml2::XMLElement* turnRangeNode = doc.NewElement("TurnRange");
        turnRangesNode->InsertEndChild(turnRangeNode);

        turnRangeNode->SetAttribute("type", turn_range.type);
        turnRangeNode->SetAttribute("start", turn_range.start_index);
        turnRangeNode->SetAttribute("end", turn_range.end_index);
    }
    } //end TurnRanges

    {//SpeedRanges
        tinyxml2::XMLElement* speedRangesNode = doc.NewElement("SpeedRanges");
        pathInfoNode->InsertEndChild(speedRangesNode);

        //创建Description子节点,并插入父节点
        tinyxml2::XMLElement* discriptionNode = doc.NewElement("Description");
        speedRangesNode->InsertEndChild(discriptionNode);

        tinyxml2::XMLElement* typeElement = doc.NewElement("speed");
        discriptionNode->InsertEndChild(typeElement);
        typeElement->InsertEndChild(doc.NewText("the max speed in the range of road"));

        tinyxml2::XMLElement* startElement = doc.NewElement("start");
        discriptionNode->InsertEndChild(startElement);
        startElement->InsertEndChild(doc.NewText("the start index of the given max speed"));

        tinyxml2::XMLElement* endElement = doc.NewElement("end");
        discriptionNode->InsertEndChild(endElement);
        endElement->InsertEndChild(doc.NewText("the end index of the given max speed"));

        tinyxml2::XMLElement* addEle = doc.NewElement("add");
        discriptionNode->InsertEndChild(addEle);
        addEle->InsertEndChild(doc.NewText("To add a max speed range manually, please follow the format below"));

        //添加属性,示例
        for(const SpeedRange& speed_range: speed_ranges_)
        {
            //创建speedRange节点
            tinyxml2::XMLElement* speedRangeNode = doc.NewElement("SpeedRange");
            speedRangesNode->InsertEndChild(speedRangeNode);

            speedRangeNode->SetAttribute("speed", speed_range.speed);
            speedRangeNode->SetAttribute("start", speed_range.start_index);
            speedRangeNode->SetAttribute("end", speed_range.end_index);
        }
    }//end SpeedRanges

    {//TrafficLightPoints
        tinyxml2::XMLElement* pointsNode = doc.NewElement("TrafficLightPoints");
        pathInfoNode->InsertEndChild(pointsNode);

        //创建Description子节点,并插入父节点
        tinyxml2::XMLElement* discriptionNode = doc.NewElement("Description");
        pointsNode->InsertEndChild(discriptionNode);

        tinyxml2::XMLElement* typeElement = doc.NewElement("index");
        discriptionNode->InsertEndChild(typeElement);
        typeElement->InsertEndChild(doc.NewText("the traffic light position in global path"));

        tinyxml2::XMLElement* startElement = doc.NewElement("id");
        discriptionNode->InsertEndChild(startElement);
        startElement->InsertEndChild(doc.NewText("the sequence of the traffic light point"));

        tinyxml2::XMLElement* addEle = doc.NewElement("add");
        discriptionNode->InsertEndChild(addEle);
        addEle->InsertEndChild(doc.NewText("To add a max speed range manually, please follow the format below"));

        //添加属性
        for(int id=0; id<traffic_light_points_.size(); ++id)
        {
            //创建point节点
            const TrafficLightPoint& point = traffic_light_points_[id];
            tinyxml2::XMLElement* pointNode = doc.NewElement("TrafficLightPoint");
            pointsNode->InsertEndChild(pointNode);

            pointNode->SetAttribute("id", id);
            pointNode->SetAttribute("index", point.index);
        }
    }//end SpeedRanges

    //6.保存xml文件
    doc.SaveFile(file_name.c_str());
}

void RecordPath::setTurnRange(const std::string& type, size_t startIdx, size_t endIdx)
{
    if(endIdx - startIdx < 5)
    {
        log("INFO","[Turn Range] Start to end is too near!");
        return;
    }

    int int_type;
    if(type == "left")
        int_type = -1;
     else if(type == "right")
        int_type = 1;
     else
        int_type = 0;

    turn_ranges_.emplace_back(int_type, startIdx, endIdx);
}

void RecordPath::setParkPoint(size_t duration)
{
    std::lock_guard<std::mutex> lck(mutex_);
    if(path_points_.empty())
    {
        log("WARN","[Parking Point] No Valid Path Points!");
        return ;
    }

    path_points_.push_back(current_point_); //强制保存当前点，以提高定点停车准确度
    //当在小范围内多次请求记录停车点时，新请求覆盖旧请求
    //vector::back()使用需谨慎，若vector为空，将访问非法内存，导致程序异常崩溃。 vector::front同理
    if(park_points_.size() >0 && fabs(park_points_.back().index - path_points_.size()) < 5)
        park_points_.back() = ParkingPoint(path_points_.size()-1, duration); //覆盖最后一个停车点
    else
        park_points_.emplace_back(path_points_.size()-1, duration);
    log("INFO","[Parking Point] Recorded.");
}

void RecordPath::setTrafficLightPoint()
{
    std::lock_guard<std::mutex> lck(mutex_);
    if(path_points_.empty())
    {
        log("WARN","[Traffic Light Point] No Valid Path Points!");
        return ;
    }

    if(traffic_light_points_.size() >0 && fabs(traffic_light_points_.back().index - path_points_.size()) < 5)
        traffic_light_points_.back() = TrafficLightPoint(path_points_.size()-1, -1);
    else
        traffic_light_points_.emplace_back(path_points_.size()-1, -1);
    log("INFO","[Traffic Light Point] Recorded.");
}

void RecordPath::setMaxSpeed(float speed, bool is_start)
{
    std::lock_guard<std::mutex> lck(mutex_);

    if(is_start)
    {
        speed_ranges_.push_back(SpeedRange(speed, path_points_.size()-1, path_points_.size()-1));
    }
    else
    {
        speed_ranges_.back().end_index = path_points_.size()-1;
    }
    log("INFO",QString("[Max Speed] %1 km/h.").arg(speed).toStdString());
}
