/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/


#include <QtGui>
#include <QMessageBox>
#include <QFileDialog>
#include <iostream>
#include "../include/main_window.hpp"
#include "unistd.h"
#include "iostream"
#include "cstdio"
#include "QDir"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace av_console {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
    QMainWindow(parent),
    qnode(argc,argv),
    m_nodeInited(false)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tabWidget->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    initSensorStatusWidget();
}

void MainWindow::initSensorStatusWidget()
{
    ui.widget_rtkStatus->setChecked(false);
    ui.widget_rtkStatus->setButtonStyle(ImageSwitch::ButtonStyle_4);
    ui.widget_rtkStatus->setClickedDisable();

    ui.widget_camera1Status->setChecked(false);
    ui.widget_camera1Status->setButtonStyle(ImageSwitch::ButtonStyle_4);
    ui.widget_camera1Status->setClickedDisable();

    ui.widget_esrStatus->setChecked(false);
    ui.widget_esrStatus->setButtonStyle(ImageSwitch::ButtonStyle_4);
    ui.widget_esrStatus->setClickedDisable();

    ui.widget_gpsStatus->setChecked(false);
    ui.widget_gpsStatus->setButtonStyle(ImageSwitch::ButtonStyle_4);
    ui.widget_gpsStatus->setClickedDisable();

    ui.widget_lidarStatus->setChecked(false);
    ui.widget_lidarStatus->setButtonStyle(ImageSwitch::ButtonStyle_4);
    ui.widget_lidarStatus->setClickedDisable();

    connect(&qnode,SIGNAL(sensorStatusChanged(int,bool)),this,SLOT(sensorStatusChanged(int,bool)));
}
void MainWindow::sensorStatusChanged(int sensor_id, bool status)
{
    //qDebug() <<"sensorStatusChanged  " <<  sensor_id << "\t " << status;
    if(qnode.Sensor_Camera1 == sensor_id)
        ui.widget_camera1Status->setChecked(status);
    else if(qnode.Sensor_Rtk == sensor_id)
        ui.widget_rtkStatus->setChecked(status);
    else if(qnode.Sensor_Lidar == sensor_id)
        ui.widget_lidarStatus->setChecked(status);
    else if(qnode.Sensor_Gps == sensor_id)
        ui.widget_gpsStatus->setChecked(status);
    else if(qnode.Sensor_Esr == sensor_id)
        ui.widget_esrStatus->setChecked(status);
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    //close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check )
{
    if ( ui.checkbox_use_environment->isChecked() )
    {
        if ( !qnode.init() )
        {
			showNoMasterMessage();
            return;
        }
        else
			ui.button_connect->setEnabled(false);
    }
    else
    {
        if ( !qnode.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString()) )
        {
            showNoMasterMessage();
            return;
        }
        else
        {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
		}
	}
    qnode.log(QNode::Info,"connect to ros master ok!");
    m_nodeInited = true;


}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}


void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}


void MainWindow::ReadSettings() {
    //目录,文件名,linux保存在~/.config
    QSettings settings("av_console", "av_console");
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
    m_pathFileDir = settings.value("pathFileDir","").toString();
    ui.lineEdit_roadNet->setText(m_pathFileDir);

    int speedIndex = settings.value("speedIndex","0").toInt();
    ui.comboBox_driverSpeed->setCurrentIndex(speedIndex);
}

void MainWindow::WriteSettings() {
    QSettings settings("av_console", "av_console");

    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry()); //保存各窗口尺寸
    settings.setValue("windowState", saveState()); //保存各窗口位置
    settings.setValue("pathFileDir",m_pathFileDir);
    settings.setValue("speedIndex",QString::number(ui.comboBox_driverSpeed->currentIndex()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace av_console


void av_console::MainWindow::on_button_roscore_clicked()
{
    if ( ros::master::check())
    {
        ui.statusbar->showMessage("roscore is already running...",1000);
        return ;
    }
    system("gnome-terminal -x roscore");
}

void av_console::MainWindow::on_pushButton_gps_clicked(bool checked)
{
    if(checked)
    {
      changeToCmdDir();
      system("gnome-terminal -e ./gps.sh");
    }
    else
    {
      changeToCmdDir();
      system("gnome-terminal -e ./kill_gps.sh");
    }
}

void av_console::MainWindow::on_pushButton_rtk_clicked(bool checked)
{
    if(checked)
    {
      changeToCmdDir();
      system("gnome-terminal -e ./rtk.sh");
      ui.lineEdit_rtk->setText("ok");
    }
    else
    {
      ui.lineEdit_rtk->setText("");
    }
}

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
          //qnode.log(qnode.Error, std::string(buf));
          qnode.log(qnode.Error,"change to cmd directory failed!");
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
  qnode.log(qnode.Info,cmdDir.absolutePath().toStdString());

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
            changeToCmdDir();
            system("gnome-terminal -e ./gps.sh");
            m_pathRecorder->log("INFO","No Location Message Published, Starting GPS Automatically.");
        }
        ui.pushButton_pathPlanning->setText("Stop And Save");
    }
    else
    {
        m_pathRecorder->stop();
        if(m_pathRecorder->pathPointsSize() < 10)
        {
          m_pathRecorder->log("WARN","path points is too few!");
          ui.pushButton_pathPlanning->setText("Start");
          return ;
        }

        if(m_pathFileDir.isEmpty())
            m_pathFileDir = "./";

        while(true)
        {
            QString fileName = QFileDialog::getSaveFileName(this,
                                        "save path points", m_pathFileDir, "TXT(*txt)");
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
            m_pathRecorder->savePathPoints(fileName.toStdString());
            delete m_pathRecorder;
            m_pathRecorder = NULL;
            m_pathFileDir = fileName;
            break;
        }
        ui.pushButton_pathPlanning->setText("Start");
    }
}

void av_console::MainWindow::showLog_in_pathPlanning(const QString& msg)
{

}

void av_console::MainWindow::updatePathPlanningLoggingView()
{
  ui.listView_pathPlanning->scrollToBottom();
}

void av_console::MainWindow::on_pushButton_openRoadNet_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                "open roadnet file", m_pathFileDir, "TXT(*txt)");
    if(fileName.isEmpty())
        return;
/*
    QStringList list = fileName.split('/');
    QString name = *(list.end()-1);
    qDebug() << fileName << "\t" << name;
    ui.lineEdit_roadNet->setText(name);
*/
    ui.lineEdit_roadNet->setText(fileName);
    m_pathFileDir = fileName;
}

void av_console::MainWindow::on_pushButton_driverlessStart_clicked(bool checked)
{
    if(checked)
    {
        if(!qnode.initialed())
        {
            ui.pushButton_driverlessStart->setChecked(false);
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.setText("Please Connect Firstly.");
            msgBox.exec();
            return;
        }

        bool ok;
        float speed = ui.comboBox_driverSpeed->currentText().toFloat(&ok);
        if(!ok)
        {
            speed = 10.0;
            //ui.lineEdit_driverlessSpeed->setText("10.0");
        }
        QString roadnet_file = ui.lineEdit_roadNet->text();
        if(roadnet_file.isEmpty())
        {
            QMessageBox msgBox;
            msgBox.setText("No Roadnet File.");
            msgBox.exec();
            ui.pushButton_driverlessStart->setChecked(false);
            return;
        }

        if(!changeToCmdDir())
        {
            ui.pushButton_driverlessStart->setChecked(false);
            return;
        }

        //询问是否保存日志文件
        QFileInfo roadnetFileInfo(roadnet_file);
        QDir roadnetDir(roadnetFileInfo.absolutePath());//文件所在目录

        QString question = tr("Save log file in ") + roadnetFileInfo.absolutePath() + tr(" ?");
        QMessageBox msgBox(QMessageBox::Question, tr("Start driverless"), question,
                           QMessageBox::YesAll|QMessageBox::Yes|QMessageBox::Cancel);
        msgBox.button(QMessageBox::YesAll)->setText(tr("Run and save"));
        msgBox.button(QMessageBox::Yes)->setText(tr("Run without save"));
        msgBox.button(QMessageBox::Cancel)->setText(tr("Cancel"));
        msgBox.setDefaultButton(QMessageBox::Yes);

        int button = msgBox.exec();
        //若点击了叉号，则放弃操作 Cancel
        //std::cout  << std::hex << button << std::endl;
        if(button == QMessageBox::Cancel)
        {
            ui.pushButton_driverlessStart->setChecked(false);
            return;
        }
        else if(button == QMessageBox::YesAll) // saveLog
        {
            //新建日志目录，启动记录程序
            std::stringstream logFilePath;
            std::time_t time = std::time(0);
            logFilePath << "driverless_data/"
                        << std::put_time(std::localtime(&time),"%Y-%m-%d-%H-%M-%S");

            roadnetDir.mkpath(tr(logFilePath.str().c_str()));

            std::stringstream cmd;
            cmd << "gnome-terminal -e './save_log.sh "
                << roadnetDir.absolutePath().toStdString() << "\/" << logFilePath.str()
                << "\/" << roadnetFileInfo.baseName().toStdString()
                <<"_log.txt'";
            std::cout  << cmd.str() << std::endl;
            system(cmd.str().c_str());
        }

        std::stringstream cmd;
        cmd << "gnome-terminal -e './driverless.sh "
            << roadnet_file.toStdString() << " " << speed << "'";
        std::cout  << cmd.str() << std::endl;
        system(cmd.str().c_str());

        ui.pushButton_driverlessStart->setText("Stop");
    }
    else
    {
        changeToCmdDir();
        system("gnome-terminal -e './stop_driverless.sh' ");
        ui.pushButton_driverlessStart->setText("Start");
    }
}

void av_console::MainWindow::on_tabWidget_currentChanged(int index)
{
    if(qnode.initialed())
        return;
    ui.tabWidget->setCurrentIndex(0);
    ui.statusbar->showMessage("please connect to master firstly!",3000);
}
