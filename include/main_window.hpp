/**
 * @file /include/av_console/main_window.hpp
 *
 * @brief Qt based gui for av_console.
 *
 * @date November 2010
 **/
#ifndef av_console_MAIN_WINDOW_H
#define av_console_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "recordpath.hpp"
#include <QFileInfo>
#include<ctime>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace av_console {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void showNoMasterMessage();
    bool changeToCmdDir(bool mode=false);
    void initSensorStatusWidget();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    void updateLoggingView(); // no idea why this can't connect automatically
    void updatePathPlanningLoggingView();
    void sensorStatusChanged(int,bool);
    void closeEvent(QCloseEvent *event);

private Q_SLOTS:
    void on_pushButton_gps_clicked(bool checked);
    void on_pushButton_rtk_clicked(bool checked);
    void on_pushButton_pathPlanning_clicked(bool checked);
    void showLog_in_pathPlanning(const QString& msg);
    void on_pushButton_openRoadNet_clicked();
    void on_pushButton_driverlessStart_clicked(bool checked);
    void on_tabWidget_currentChanged(int index);

private:
    Ui::MainWindow ui;
    QNode qnode;
    RecordPath *m_pathRecorder;
    bool m_nodeInited;
    QString m_pathFileDir;

};

}  // namespace av_console

#endif // av_console_MAIN_WINDOW_H
