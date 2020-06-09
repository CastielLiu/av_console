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
#include "../include/av_console/main_window.hpp"
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
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
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
    QSettings settings("Qt-Ros Package", "av_console");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://127.0.0.1:11311/")).toString();
    QString host_url = settings.value("host_url", QString("127.0.0.1")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);

    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
    m_pathFileDir = settings.value("pathFileDir","").toString();

}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "av_console");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("pathFileDir",m_pathFileDir);

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace av_console


void av_console::MainWindow::on_button_roscore_clicked()
{
    if ( ros::master::check() )
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

bool av_console::MainWindow::changeToCmdDir()
{
  static bool parsed = false;
  static QDir cmdDir;
  if(parsed)
  {
    QDir::setCurrent(cmdDir.absolutePath()); //切换目录
    return true;
  }

  FILE * fp =  popen("rospack find av_console", "r");
  char buf[50] ;
  fscanf(fp,"%s",buf);
  pclose(fp);
  if(std::string(buf).find("home") == std::string::npos)
  {
      //qnode.log(qnode.Error, std::string(buf));
      qnode.log(qnode.Error,"change to cmd directory failed!");
      return false;
  }

  cmdDir = QDir::current();//获取当前工作目录
  cmdDir.cd(QString(buf));      //修改目录，仅修改了目录名，未切换
  cmdDir.cd("cmd");
  QDir::setCurrent(cmdDir.absolutePath()); //切换目录

  parsed = true;
  return true;
}

void av_console::MainWindow::on_pushButton_pathPlanning_clicked(bool checked)
{
    if(checked)
    {
        ui.pushButton_pathPlanning->setText("Stop And Save");
        m_pathRecorder = new RecordPath();
        ui.listView_pathPlanning->setModel(m_pathRecorder->loggingModel());
        connect(m_pathRecorder, SIGNAL(loggingUpdated()), this, SLOT(updatePathPlanningLoggingView()));
        m_pathRecorder->start();
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
        bool ok;
        float speed = ui.lineEdit_driverlessSpeed->text().toFloat(&ok);
        if(!ok)
        {
            speed = 10.0;
            ui.lineEdit_driverlessSpeed->setText("10.0");
        }
        QString fileName = ui.lineEdit_roadNet->text();
        if(fileName.isEmpty())
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

        std::stringstream cmd;
        cmd << "gnome-terminal -e './driverless.sh "
            << fileName.toStdString() << " " << speed << "'";
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
