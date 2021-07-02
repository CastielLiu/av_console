
#include <QtGui>
#include <QMessageBox>
#include <QFileDialog>
#include <iostream>
#include <QFont>
#include "../include/main_window.hpp"
#include "unistd.h"
#include "iostream"
#include "cstdio"
#include <QDir>
#include "autodisapperdialog.hpp"


namespace av_console {
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QSplashScreen* splash, QWidget *parent):
    QMainWindow(parent),
    qnode(argc,argv),
    m_nodeInited(false),
    m_pathRecorder(nullptr),
    m_dataRecorder(nullptr),
    m_forceExit(false),
    m_splash(splash)
{
    //注册信号槽数据类型
    qRegisterMetaType<std::string>("std_string");

    ui.setupUi(this);

    m_splash->setFont(QFont("microsoft yahei", 20));
    m_splash->showMessage("Initializing UI ...", Qt::AlignCenter|Qt::AlignBottom);

    ReadSettings();
    //setWindowIcon(QIcon(":/images/icon.png"));
    ui.tabWidget->setCurrentIndex(0);

    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(taskStateChanged(int,const QString&)), this, SLOT(onTaskStateChanged(int,const QString&)));
    QObject::connect(&qnode, SIGNAL(rosmasterOffline()), this, SLOT(onRosmasterOffline()));
    QObject::connect(&mTimer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    QObject::connect(&qnode, SIGNAL(showDiagnosticMsg(const QString&,int,const QString&)),
                     this, SLOT(onShowDiagnosticMsg(const QString&,int,const QString&)));

    this->initSensorStatusWidget(); //初始化传感器状态widget

    //设置treeWidget_diagnosics
    //如设置为根据文本内容调整尺寸，将导致加载速度变慢
    //ui.treeWidget_diagnosics->header()->setResizeMode(QHeaderView::ResizeToContents); //根据内容调整尺寸
    ui.treeWidget_diagnosics->setUniformRowHeights(true); //使用同一高度，防止每次加载新数据都计算行高，以加快载入

    if(!initDriverlessSystem())
    {
        //延时以充分显示错误信息
        std::this_thread::sleep_for(std::chrono::seconds(2));
        exit(0);
    }
}

MainWindow::~MainWindow()
{
    if(m_dataRecorder != nullptr)
        delete m_dataRecorder;
}

void MainWindow::customizeButtonsClicked(bool checked)
{
    QPushButton *button = qobject_cast<QPushButton *>(sender());
    //qDebug() << button->text();
    launchRosNodes(button->text().toStdString(), !checked);
}

bool MainWindow::initDriverlessSystem()
{
    m_splash->showMessage("Preparing  driverless ...", Qt::AlignCenter|Qt::AlignBottom);

    if(!initQnode()) return false;

    if(!qnode.waitForDriverlessServer(3.0))
    {
        m_splash->showMessage("Connect to driverless server failed!",Qt::AlignCenter|Qt::AlignBottom, Qt::red);
        return false;
    }

    qnode.start();

    m_splash->showMessage("Initializing complete!",Qt::AlignCenter|Qt::AlignBottom, Qt::green);

    return true;
}

/*初始化传感器状态显示控件*/
void MainWindow::initSensorStatusWidget()
{
    //+ 添加自定义按钮
    for (auto iter = g_rosNodesArray.begin(); iter != g_rosNodesArray.end(); iter++)
    {
         RosNodes node = iter->second;
         std::string node_name = iter->first;
         if(node.use_button)
         {
              QPushButton * button = new QPushButton(ui.scrollArea_customizeButtons);
              button->setCheckable(true);
              button->setObjectName(QString("customized_button_%1").arg(node_name.c_str()));
              button->setText(QString::fromStdString(node_name));
              ui.customizeButtonsLayout->addWidget(button);
              connect(button,SIGNAL(clicked(bool)),this, SLOT(customizeButtonsClicked(bool)));
         }
    }
    QSpacerItem* spacerItem = new QSpacerItem(10,10,QSizePolicy::Minimum, QSizePolicy::Expanding);
    ui.customizeButtonsLayout->addItem(spacerItem);
    //- 添加自定义按钮

    //+ 添加传感器状态显示
    bool is_first_sensor = true;
    for (auto iter = g_rosNodesArray.begin(); iter != g_rosNodesArray.end(); iter++)
    {
         const RosNodes& node = iter->second;
         const std::string& node_name = iter->first;
         int node_id = node.id;
         if(node.show_status)
         {
             //label
             QLabel * statusLabel = new QLabel(node_name.c_str(), this);
             QFont font; font.setPointSize(FontLevel3);
             statusLabel->setFont(font);
             //led
             ImageSwitch* statusWidget = new ImageSwitch(ui.groupBox_sensorStatus);
             statusWidget->setObjectName(QString("widgetImageSwitch_")+node_name.c_str());
             statusWidget->setChecked(false);
             statusWidget->configButton(ImageSwitch::ButtonStyle_4);
             statusWidget->setClickedDisable();

             if(!is_first_sensor)
             {
                 QSpacerItem* spacerItem = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
                 ui.horizontalLayout_sensorStatus->addItem(spacerItem);
             }
             is_first_sensor = false;

             ui.horizontalLayout_sensorStatus->addWidget(statusWidget);
             ui.horizontalLayout_sensorStatus->addWidget(statusLabel);

             m_sensorStatusWidgets[node_id] = statusWidget;
             qnode.addSensor(node_id, node_name, node.topics.begin()->second);
         }
    }
    //- 添加传感器状态显示

    connect(&qnode,SIGNAL(sensorStatusChanged(int,bool)),this,SLOT(sensorStatusChanged(int,bool)));
}

//start driverless
void av_console::MainWindow::on_pushButton_driverlessStart_clicked(bool checked)
{
    if(checked)
    {
        //确保qnode已经初始化
        if(!qnode.initialed())
        {
            onTaskStateChanged(qnode.Driverless_Idle);
            QMessageBox msgBox(this);
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.setText("Please Connect Firstly.");
            msgBox.exec();
            return;
        }

        if(!qnode.serverConnected())
        {
            //this->showMessgeInStatusBar("driverless server is not connected! restarting driverless program...", true);
            qnode.stampedLog(qnode.Info, "driverless server is not connected!\n automatic starting driverless program...");
            launchRosNodes("driverless");
            onTaskStateChanged(qnode.Driverless_Idle);
            return;
        }

        bool ok;
        float speed = ui.comboBox_driverSpeed->currentText().toFloat(&ok);
        QString roadnet_file = m_roadNetFileDir; //ui.lineEdit_roadNet->text();
        if(roadnet_file.isEmpty())
        {
            QMessageBox msgBox(this);
            msgBox.setText("No Roadnet File.");
            msgBox.exec();
            onTaskStateChanged(qnode.Driverless_Idle);
            return;
        }

        //询问是否保存日志文件
        QFileInfo roadnetFileInfo(roadnet_file);
        QDir roadnetDir(roadnetFileInfo.absolutePath());//文件所在目录

        QString question = tr("Save log file in ") + roadnetFileInfo.absolutePath() + tr(" ?");
        QMessageBox msgBox(QMessageBox::Question, tr("Start driverless"), question,
                           QMessageBox::YesAll|QMessageBox::Yes|QMessageBox::Cancel, this);

        msgBox.button(QMessageBox::YesAll)->setText(tr("Run and save"));
        msgBox.button(QMessageBox::Yes)->setText(tr("Run without save"));
        msgBox.button(QMessageBox::Cancel)->setText(tr("Cancel"));
        msgBox.setDefaultButton(QMessageBox::Yes);
        int button = msgBox.exec();
        //若点击了叉号，则放弃操作 Cancel
        //std::cout  << std::hex << button << std::endl;
        if(button == QMessageBox::Cancel)
        {
            onTaskStateChanged(qnode.Driverless_Idle);
            return;
        }
        else if(button == QMessageBox::YesAll) // saveLog
        {
        }

        driverless_actions::DoDriverlessTaskGoal goal;
        goal.roadnet_file = roadnet_file.toStdString();
        goal.expect_speed = speed;

        //目标类型
        if(this->ui.comboBox_goalType->currentText() == "Path")  //路径文件
            goal.type = goal.FILE_TYPE;
        else if(this->ui.comboBox_goalType->currentText() == "Goal") //目标点
        {
            goal.type = goal.POSE_TYPE;
            PoseArray path;
            if(!loadPathPoints(roadnet_file.toStdString(), path) || path.poses.size() < 1)
            {
                QMessageBox msgBox;
                msgBox.setText(QString("Load goal pose from %1 failed!").arg(roadnet_file));
                msgBox.exec();
                onTaskStateChanged(qnode.Driverless_Idle);
                return;
            }
            const Pose goal_pose = path.poses[0];
            goal.target_pose.x = goal_pose.x;
            goal.target_pose.y = goal_pose.y;
            goal.target_pose.theta = goal_pose.yaw;
        }
        else
        {
            QMessageBox::warning(this,"Unknown Road Type!", "Unknown Road Type!");
            onTaskStateChanged(qnode.Driverless_Idle);
            return;
        }
        //任务类型
        if(ui.comboBox_taskType->currentText() == "Drive") //前进
            goal.task  = goal.DRIVE_TASK;
        else if(ui.comboBox_taskType->currentText() == "Reverse")
        {
            goal.task  = goal.REVERSE_TASK;
            if(this->ui.checkBox_pathFilp->checkState() == Qt::Checked)
                goal.path_filp = true;
            else
                goal.path_filp = false;
        }

        qnode.requestDriverlessTask(goal);
        onTaskStateChanged(qnode.Driverless_Starting);
    }
    else
    {
        qnode.cancleAllGoals();
        onTaskStateChanged(qnode.Driverless_Idle);
    }
}

//connect
void MainWindow::on_pushButton_connect_clicked()
{
    initQnode();
}

bool MainWindow::initQnode()
{
    std::string master = "";
    std::string host   = "";
    if(!ui.checkbox_use_environment->isChecked())
    {
        master = ui.line_edit_master->text().toStdString();
        host   = ui.line_edit_host->text().toStdString();
    }
    ui.pushButton_connect->setEnabled(false);

    if (!qnode.init(5, master, host))
    {
        m_splash->showMessage("Detect rosmaster failed!",Qt::AlignCenter|Qt::AlignBottom, Qt::red);
        return false;
    }

    m_nodeInited = true;
    return true;
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
    ui.line_edit_master->setReadOnly(state);
    ui.line_edit_host->setReadOnly(state);
}


void MainWindow::sensorStatusChanged(int sensor_id, bool status)
{
    qDebug() <<"sensorStatusChanged  " <<  sensor_id << "\t " << status;
    m_sensorStatusWidgets[sensor_id]->setChecked(status);
}

void MainWindow::showNoMasterMessage()
{
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
}

void MainWindow::showMessgeInStatusBar(const QString& msg, bool warnning)
{
    ui.statusbar->showMessage(msg, 3000);
    if(warnning)
        ui.statusbar->setStyleSheet("color: red");
    else
        ui.statusbar->setStyleSheet("color: black");
}

void MainWindow::updateLoggingView()
{
   ui.view_logging->scrollToBottom();
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About ..."),tr("SEU Automatic Vehicle Console."));
}

void MainWindow::on_actionEditCommandFile_triggered()
{
    mCmdFileName = QCoreApplication::applicationDirPath() + "/../cmd/cmd.xml";
    if(mCmdFileName.isEmpty())
    {
        QMessageBox::critical(this,"Error","Command File Not Exist!");
        return ;
    }
    QString cmd = QString("gedit ") + mCmdFileName + "&";
    system(cmd.toStdString().c_str());

}

void MainWindow::ReadSettings() {
    QString app_dir = QCoreApplication::applicationDirPath();
    QString cfg_file = app_dir + "/../cfg/cfg.ini";

    QSettings settings(cfg_file, QSettings::IniFormat);

    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://127.0.0.1:11311/")).toString();
    QString host_url = settings.value("host_url", QString("127.0.0.1")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);

    bool checked = settings.value("use_environment_variables", true).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
    m_roadNetFileDir = settings.value("roadNetFileDir","").toString();
    ui.lineEdit_roadNet->setText(getShortFileName(m_roadNetFileDir));

    int speedIndex = settings.value("speedIndex","0").toInt();
    ui.comboBox_driverSpeed->setCurrentIndex(speedIndex);

    m_recordFileDir = settings.value("recordDataFileDir","").toString();
    ui.lineEdit_recordFileName->setText(m_recordFileDir);
}

void MainWindow::WriteSettings() {
    QString app_dir = QCoreApplication::applicationDirPath();
    QString cfg_file = app_dir + "/../cfg/cfg.ini";

    QSettings settings(cfg_file, QSettings::IniFormat);

    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry()); //保存各窗口尺寸
    settings.setValue("windowState", saveState()); //保存各窗口位置
    settings.setValue("roadNetFileDir",m_roadNetFileDir);
    settings.setValue("speedIndex",QString::number(ui.comboBox_driverSpeed->currentIndex()));

    settings.setValue("recordDataFileDir",m_recordFileDir);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();

    if(!m_forceExit)
    {
        int button = QMessageBox::question(this,"Confirm Exit",
                                       "Are you sure to exit??",
                                       QMessageBox::Yes|QMessageBox::No,
                                       QMessageBox::No);
        if(button != QMessageBox::Yes)
        {
            event->ignore();
            return ;
        }
    }

    launchRosNodes("driverless", true); //close driverless
    event->accept();
    QMainWindow::closeEvent(event);
}

}  // namespace av_console

//mode=true  ros 工作空间目录
//mode=false 应用程序所在目录 default
bool av_console::MainWindow::changeToCmdDir(bool mode)
{
  static bool parsed = false;
  static QDir cmdDir;
  if(parsed)
  {
    QDir::setCurrent(cmdDir.absolutePath()); //切换目录
    return true;
  }

  QString cmdPath;
  if(mode)
  {
      char buf[50] ;
      FILE * fp =  popen("rospack find av_console", "r");
      fscanf(fp,"%s",buf);
      pclose(fp);
      if(std::string(buf).find("home") == std::string::npos)
      {
          qnode.stampedLog(qnode.Error,"change to cmd directory failed!");
          return false;
      }
      cmdPath = tr(buf);
  }
  else
  {
      //可执行程序所在目录
      //std::cout << QCoreApplication::applicationDirPath().toStdString() << std::endl;
      cmdPath = QCoreApplication::applicationDirPath();
  }

  cmdDir = QDir::current();//获取当前工作目录
  cmdDir.cd(cmdPath);      //修改目录，仅修改了目录名，未切换
  cmdDir.cd("../cmd");
  QDir::setCurrent(cmdDir.absolutePath()); //切换目录

  parsed = true;
  return true;
}

void av_console::MainWindow::on_pushButton_pathPlanning_clicked(bool checked)
{
    if(checked)
    {
        m_pathRecorder = new RecordPath();
        ui.listView_pathPlanning->setModel(m_pathRecorder->loggingModel());
        connect(m_pathRecorder, SIGNAL(loggingUpdated()), this, SLOT(updatePathPlanningLoggingView()));
        if(!m_pathRecorder->start())
        {
            ui.pushButton_pathPlanning->setChecked(false);
            return;
        }
        ui.pushButton_pathPlanning->setText("Stop And Save");
        ui.groupBox_pathPlanConfig->setDisabled(false);
    }
    else
    {
        ui.groupBox_pathPlanConfig->setDisabled(true);
        m_pathRecorder->stop();
        if(m_pathRecorder->pathPointsSize() == 0)
        {
          m_pathRecorder->log("WARN","path points is too few !");
          ui.pushButton_pathPlanning->setText("Start");
          return ;
        }

        if(m_recordFileDir.isEmpty())
            m_recordFileDir = "./";

        while(true)
        {
            QString fileName = QFileDialog::getSaveFileName(this,
                                        "save path points", m_recordFileDir, "TXT(*txt)");
            if(fileName.isEmpty())
            {
                int Abandon =
                QMessageBox::question(this,"question","Abandon this record?",
                                      QMessageBox::Yes | QMessageBox::No,QMessageBox::No);
                if(Abandon == QMessageBox::Yes)
                {
                    delete m_pathRecorder;
                    m_pathRecorder = NULL;
                    break;
                }
                else
                    continue;

            }
            std::string pathInfoFile =
            fileName.toStdString().substr(0,fileName.toStdString().find_last_of(".")) + "_info.xml";
            m_pathRecorder->generatePathInfoFile(pathInfoFile);
            m_pathRecorder->savePathPoints(fileName.toStdString());
            delete m_pathRecorder;
            m_pathRecorder = NULL;
            break;
        }
        ui.pushButton_pathPlanning->setText("Start");
    }
}

void av_console::MainWindow::updatePathPlanningLoggingView()
{
  ui.listView_pathPlanning->scrollToBottom();
}

void av_console::MainWindow::on_pushButton_openRoadNet_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                "open roadnet file", m_roadNetFileDir, "TXT(*txt)");
    if(fileName.isEmpty())
        return;

    m_roadNetFileDir = fileName;

    //qDebug() << fileName << "\t" << name;
    ui.lineEdit_roadNet->setText(getShortFileName(m_roadNetFileDir));
}

void av_console::MainWindow::on_tabWidget_currentChanged(int index)
{
    static int last_index = 0;

    if(!qnode.initialed())
    {
        if(index != stackWidgetIndex_driverless)
        {
            showMessgeInStatusBar("please connect to master firstly!", true);
            g_logg << "qnode not initial!\n";
            //qnode.log("please connect to master firstly!");
            ui.tabWidget->setCurrentIndex(stackWidgetIndex_driverless);
        }
        return;
    }
    if(index == stackWidgetIndex_recorder)
    {
        bool ok;
        QString text = QInputDialog::getText(this, "Infomation", "This feature is under development",
                                                     QLineEdit::Normal,
                                                     "",&ok);

        if(m_dataRecorder == nullptr)
        {
            //实例化数据记录器
            m_dataRecorder = new RecordData();
        }

        if(text == "seucar")
            m_dataRecorder->setDisable(false);
        else
            m_dataRecorder->setDisable(true);
    }

    last_index = index;
}

void av_console::MainWindow::onTaskStateChanged(int state, const QString& info)
{
    if(!info.isEmpty())
    {
        AutoDisapperDialog* dialog =
                new AutoDisapperDialog(this, QMessageBox::Information,info,2000);
    }

    if(state == qnode.Driverless_Idle)
    {
        ui.pushButton_driverlessStart->setChecked(false);
        ui.pushButton_driverlessStart->setText("Start");
    }
    else if(state == qnode.Driverless_Starting)
    {
        ui.pushButton_driverlessStart->setChecked(true);
        ui.pushButton_driverlessStart->setText("Starting");
    }
    else if(state == qnode.Driverless_Running)
    {
        ui.pushButton_driverlessStart->setChecked(true);
        ui.pushButton_driverlessStart->setText("Stop");
    }
    else if(state == qnode.Driverless_Complete)
    {
        ui.pushButton_driverlessStart->setChecked(false);
        ui.pushButton_driverlessStart->setText("Start");
    }
    else if(state == qnode.Driverless_Offline)
    {
        //QObject::disconnect(&qnode, SIGNAL(taskStateChanged(int)), this, SLOT(onTaskStateChanged(int)));
        static bool showing = false;

        //若提示弹窗正在显示，直接返回，防止多次弹出
        if(showing) return;
        showing = true;

        ui.pushButton_driverlessStart->setChecked(false);
        ui.pushButton_driverlessStart->setText("Start");

        QMessageBox msgBox(QMessageBox::Warning, tr("Driverless Server Offline!"), tr("Close Application or Restart?"),
                           QMessageBox::Yes|QMessageBox::Cancel);

        msgBox.button(QMessageBox::Yes)->setText(tr("restart"));
        msgBox.button(QMessageBox::Cancel)->setText(tr("close"));

        msgBox.setDefaultButton(QMessageBox::Cancel);

        int button = msgBox.exec();

        //点击叉号或者Cancel均返回QMessageBox::Cancel
        if(button == QMessageBox::Yes)
            qApp->exit(777); //restart
        else
            qApp->exit(0);

        showing = false;
    }
}

void av_console::MainWindow::on_comboBox_taskType_activated(const QString &arg1)
{
    if(arg1 == "Custom") //自定义任务形式
    {
        //if(m_customDialog == nullptr)
         //   m_customDialog = new CustomTaskDialog(this);

    }
}

//ros master shutdown , reEnable the connect button
void av_console::MainWindow::onRosmasterOffline()
{
    //ui.pushButton_connect->setEnabled(true);
    //创建msgBox
    QMessageBox msgBox(QMessageBox::Warning, tr("ROS Master Shutdown!"), tr("Close Application or Restart?"),
                       QMessageBox::Yes|QMessageBox::Cancel);
    //添加按键并重命名
    msgBox.button(QMessageBox::Yes)->setText(tr("restart"));
    msgBox.button(QMessageBox::Cancel)->setText(tr("close"));

    //设置默认按键
    msgBox.setDefaultButton(QMessageBox::Yes);
    //启动msgBox事件循环
    int button = msgBox.exec();

    //点击叉号或者Cancel均返回QMessageBox::Cancel
    if(button == QMessageBox::Yes)
        qApp->exit(777); //restart
    else
        qApp->exit(0);
}

void av_console::MainWindow::onTimeout()
{

}

void av_console::MainWindow::showEvent(QShowEvent* event)
{
   this->setPushButtonStylesheet(QString("font: 14pt \"Sans Serif\";"));
}

/*@brief 配置所有pushButton的stylesheet
 */
void av_console::MainWindow::setPushButtonStylesheet(const QString& style)
{
    ui.pushButton_connect->setStyleSheet(style);
    ui.pushButton_driverlessStart->setStyleSheet(style);
    ui.pushButton_openRoadNet->setStyleSheet(style);
    ui.pushButton_pathPlanning->setStyleSheet(style);
    ui.pushButton_selectRecordFile->setStyleSheet(style);
    ui.pushButton_startRecordData->setStyleSheet(style);

    ui.groupBox_sensorStatus->setTitle("");
    //设置等高
    ui.pushButton_quit->setFixedHeight(ui.groupBox_sensorStatus->height());
    ui.pushButton_startRecordData->setCheckable(true);
}

//获取QObject控件下的所有子控件
QObjectList av_console::MainWindow::getAllLeafChilds(QObject* object)
{
    QObjectList result;
    std::queue<QObject *> queue;
    queue.push(object);
    while(!queue.empty())
    {
        QObject * node = queue.front(); queue.pop();
        //qDebug() << "pop " << node->objectName();

        QObjectList childs = node->children();
        if(childs.size() == 0) //无子节点，当且节点为叶子
        {
            result.push_back(node);
            continue;
        }

        //将其所有子节点放入队列
        for(QObject* child : childs)
        {
            queue.push(child);
            //qDebug() << "push " << child->objectName();
        }
    }
    return result;
}

void av_console::MainWindow::disableRecordDataConfigure(bool flag)
{
    QObjectList childs = getAllLeafChilds(ui.widget_recorderConfig);
    for(QObject* child : childs)
    {
        QCheckBox* checkBox = qobject_cast<QCheckBox*>(child);
        QPushButton* botton = qobject_cast<QPushButton*>(child);
        QLineEdit *lineEdit = qobject_cast<QLineEdit*>(child);
        if(checkBox)
            checkBox->setDisabled(flag);
        else if(botton)
            botton->setDisabled(flag);
        else if(lineEdit)
            lineEdit->setReadOnly(flag);
    }
}

void av_console::MainWindow::on_pushButton_startRecordData_clicked(bool checked)
{
    if(checked)
    {
        static bool listview_inited  = false;

        if(!listview_inited)
        {
            ui.listView_dataRecorder->setModel(m_dataRecorder->loggingModel());
            connect(m_dataRecorder, SIGNAL(loggingUpdated()), this, SLOT(updateDataRecorderLoggingView()));
            listview_inited = true;
        }

        m_dataRecorder->clearLog();
        m_dataRecorder->setDataFile(ui.lineEdit_recordFileName->text());
        m_dataRecorder->setRecordFrequency(ui.lineEdit_recordFrequency->text().toInt());
        m_dataRecorder->setLaunchSensorWaitTime(ui.lineEdit_recorderWaitTime->text().toInt());

        std::string vehicle_state_topic = g_rosNodesArray["base_control"].topics["vehicle_state"];
        std::string gps_topic = g_rosNodesArray["gps"].topics["odom"];
        std::string imu_topic = g_rosNodesArray["imu"].topics["corr_imu"];

        m_dataRecorder->setRecordVehicleState(vehicle_state_topic, ui.checkBox_recordRoadwheelAngle->isChecked(),
                                              ui.checkBox_recordSpeed->isChecked());
        m_dataRecorder->setRecordGps(gps_topic, ui.checkBox_recordYaw->isChecked(),
                                     ui.checkBox_recordWGS84->isChecked(), ui.checkBox_recordUTM->isChecked());
        m_dataRecorder->setRecordImu(imu_topic, ui.checkBox_recordAnglularVel->isChecked(),
                                     ui.checkBox_recordAccel->isChecked());

        m_dataRecorder->setRecordStamp(ui.checkBox_recordDataStamp->isChecked());
        bool ok = m_dataRecorder->start();

        if(!ok)
        {
            ui.pushButton_startRecordData->setChecked(false);
            return;
        }

        disableRecordDataConfigure(true);
        ui.pushButton_startRecordData->setText(QString("Stop"));
    }
    else
    {
        m_dataRecorder->stop();
        disableRecordDataConfigure(false);
        ui.pushButton_startRecordData->setText(QString("Start"));
    }
}

void av_console::MainWindow::updateDataRecorderLoggingView()
{
  ui.listView_dataRecorder->scrollToBottom();
}

void av_console::MainWindow::on_pushButton_selectRecordFile_clicked()
{
    if(m_recordFileDir.isEmpty())
        m_recordFileDir = "/home/";

    QString fileName = QFileDialog::getSaveFileName(this, QString("New File"), m_recordFileDir, "TXT(*txt)");
    if(fileName.isEmpty())
        return;

    ui.lineEdit_recordFileName->setText(fileName);
    m_recordFileDir = fileName;
}

void av_console::MainWindow::on_pushButton_setPathWidth_clicked()
{
    assert(m_pathRecorder);
    bool ok;
    float left = ui.lineEdit_leftRoadWidth->text().toFloat(&ok);
    if(!ok)
    {
        ui.lineEdit_leftRoadWidth->setText("Error");
        return;
    }
    float right = ui.lineEdit_rightRoadWidth->text().toFloat(&ok);
    if(!ok)
    {
        ui.lineEdit_rightRoadWidth->setText("Error");
        return;
    }
    m_pathRecorder->setRoadWidth(left, right);
}

void av_console::MainWindow::on_pushButton_setLeftTurn_clicked(bool checked)
{
    assert(m_pathRecorder);
    size_t currentIdx = m_pathRecorder->getPointsSize() - 1;
    static size_t start_turn_index;

    if(checked) //start
    {
        start_turn_index = currentIdx;
        ui.pushButton_setLeftTurn->setText("End Left Turn");
        ui.pushButton_setLeftTurn->setStyleSheet(QString::fromUtf8("font: 10pt \"Sans Serif\";color: rgb(255, 0, 0);"));

        ui.pushButton_setRightTurn->setDisabled(true);
    }
    else //end
    {
        size_t end_turn_index = currentIdx;
        m_pathRecorder->setTurnRange("left",start_turn_index, end_turn_index);
        ui.pushButton_setLeftTurn->setText("Start Left Turn");
        ui.pushButton_setLeftTurn->setStyleSheet(QString::fromUtf8("font: 10pt \"Sans Serif\";"));

        ui.pushButton_setRightTurn->setDisabled(false);
    }
}

void av_console::MainWindow::on_pushButton_setRightTurn_clicked(bool checked)
{   
    assert(m_pathRecorder);
    size_t currentIdx = m_pathRecorder->getPointsSize() - 1;
    static size_t start_turn_index;

    if(checked) //start
    {
        start_turn_index = currentIdx;
        ui.pushButton_setRightTurn->setText("End Right Turn");
        ui.pushButton_setRightTurn->setStyleSheet(QString::fromUtf8("font: 10pt \"Sans Serif\";color: rgb(255, 0, 0);"));

        ui.pushButton_setLeftTurn->setDisabled(true);
    }
    else //end
    {
        size_t end_turn_index = currentIdx;
        m_pathRecorder->setTurnRange("right",start_turn_index, end_turn_index);
        ui.pushButton_setRightTurn->setText("Start Right Turn");
        ui.pushButton_setRightTurn->setStyleSheet(QString::fromUtf8("font: 10pt \"Sans Serif\";"));

        ui.pushButton_setLeftTurn->setDisabled(false);
    }
}

void av_console::MainWindow::on_pushButton_setParkPoint_clicked()
{
    assert(m_pathRecorder);
    size_t duration = ui.lineEdit_parkDuration->text().toInt();
    if(duration <= 0)
    {
        ui.lineEdit_parkDuration->setText(QString("Error"));
        return;
    }
    m_pathRecorder->setParkPoint(duration);
/*
    static bool newOp = true;
    if(newOp)
        ui.pushButton_setParkPoint->setStyleSheet(QString::fromUtf8("font: 10pt \"Sans Serif\"; color: rgb(255, 0, 0);"));
    else
        ui.pushButton_setParkPoint->setStyleSheet(QString::fromUtf8("font: 10pt \"Sans Serif\";"));
    newOp = !newOp;
*/
}

void av_console::MainWindow::on_pushButton_setRoadMaxSpeed_clicked(bool checked)
{
    assert(m_pathRecorder);

    float speed = ui.lineEdit_roadMaxSpeed->text().toFloat();
    if(speed == 0.0)
    {
        QMessageBox::warning(this,"Warn","Road Max Speed Can Not Be Zero!");
        ui.pushButton_setRoadMaxSpeed->setChecked(false);
    }
    else
        m_pathRecorder->setMaxSpeed(speed, checked);

    ui.lineEdit_roadMaxSpeed->setEnabled(!ui.pushButton_setRoadMaxSpeed->isChecked());
}

void av_console::MainWindow::on_actionReinstall_triggered()
{
    int button = QMessageBox::question(this,"Reinstall",
                                       "The current program will exit automatically.\n Confirm the reinstall?",
                                       QMessageBox::Yes|QMessageBox::No,
                                       QMessageBox::No);
    if(button != QMessageBox::Yes)
        return ;

    QString app_dir = QCoreApplication::applicationDirPath();
    QString cmd_dir = app_dir + "/../";
    QString cmd = cmd_dir+"install.sh 1"; //执行安装程序时延迟1s再进行拷贝，确保可执行程序已经退出以防止占用导致拷贝失败
    execute_process_detached(cmd.toStdString());
    m_forceExit = true;
    this->close();
}

void av_console::MainWindow::on_pushButton_setTrafficLightPoint_clicked()
{
    m_pathRecorder->setTrafficLightPoint();
}

void av_console::MainWindow::onShowDiagnosticMsg(const QString& device, int level,const QString& msg)
{
    static int cnt = 1;
    static const std::vector<QString> levelVector = {"INFO", "WARN", "ERROR", "STALE"};
    if(level >= levelVector.size())
    {
        qDebug() << "Diagnostic level error!";
        g_logg << "Diagnostic level error!\n";
        return;
    }

    int index_of_realtime = m_realtimeDeviceList.indexOf(device);
    if(index_of_realtime != -1) //device 在实时列表中，使数据显示于
    {
        QTreeWidgetItem* item = ui.treeWidget_fixedDiagnostic->topLevelItem(index_of_realtime);
        setWidgetItemColorByMsgLevel(item, level);
        int seq = item->text(0).toInt()+1;
        item->setText(0,QString::number(seq));
        item->setText(1,device);
        item->setText(2,levelVector[level]);
        item->setText(3,msg);

        return ;
        //ui.treeWidget_fixedDiagnostic->header()
    }

    int rows = ui.treeWidget_diagnosics->topLevelItemCount();
    //当超过m行后删除n行
    if(rows > 800)
    {
        for(int i=0; i<200; ++i)
            delete ui.treeWidget_diagnosics->topLevelItem(0);
    }

    QStringList list; list << QString::number(cnt++) << device << levelVector[level] << msg;
    g_logg << device << ": " << levelVector[level] << "  " << msg << "\n";
    //qDebug() << list;

    QTreeWidgetItem* item = new QTreeWidgetItem(list);
    setWidgetItemColorByMsgLevel(item, level);

    ui.treeWidget_diagnosics->addTopLevelItem(item);
    ui.treeWidget_diagnosics->scrollToBottom();
}

//根据诊断信息的级别设置item的颜色
void av_console::MainWindow::setWidgetItemColorByMsgLevel(QTreeWidgetItem* item, int level)
{
    if(level == 2) //ERROR
    {
        item->setTextColor(2,Qt::red);
        item->setTextColor(3,Qt::red);
    }
    else if(level == 1) //WARN
    {
        //item->setBackgroundColor(2,Qt::yellow);
        //item->setBackgroundColor(3,Qt::yellow);
        item->setTextColor(2,Qt::blue);
        item->setTextColor(3,Qt::blue);
    }
}

//滚动诊断数据双击槽函数，当某个device被双击时，加入表单以显示于固定的诊断数据窗口
void av_console::MainWindow::on_treeWidget_diagnosics_itemDoubleClicked(QTreeWidgetItem *item, int column)
{
    if(column == 1) //(Device被双击)
    {
        //qDebug() << item->text(0) << " " << item->text(1) << " " << item->text(2);
        QString deviceName = item->text(column); //被双击的设备名称
        if(m_realtimeDeviceList.indexOf(deviceName) == -1) //设备不在实时显示列表，则添加进列表
        {
            m_realtimeDeviceList << deviceName;
            QStringList list;
            for(int i=0; i<item->columnCount(); ++i)
                list << item->text(i);

            //拷贝滚动显示列表信息到实时列表，
            //不能直接使用指针进行拷贝，否则导致无法显示，估计是同一个item只能添加到一个tree
            QTreeWidgetItem* new_item = new QTreeWidgetItem(list);
            ui.treeWidget_fixedDiagnostic->addTopLevelItem(new_item);
        }
    }
}

void av_console::MainWindow::on_treeWidget_fixedDiagnostic_itemDoubleClicked(QTreeWidgetItem *item, int column)
{
    if(column == 1) //Device被双击,从实时列表中删除，重新显示于滚动列表
    {
        QString deviceName = item->text(column); //被双击的设备名称
        int index_of_realtime = m_realtimeDeviceList.indexOf(deviceName);
        if(index_of_realtime == -1)
        {
            qDebug() << "Error! " << deviceName << " Must in m_realtimeDeviceList!";
            g_logg << "Error! " << deviceName << " Must in m_realtimeDeviceList!\n";
            return;
        }
        delete ui.treeWidget_fixedDiagnostic->topLevelItem(index_of_realtime);
        m_realtimeDeviceList.removeAt(index_of_realtime);
    }
}
