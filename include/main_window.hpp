
#ifndef av_console_MAIN_WINDOW_H
#define av_console_MAIN_WINDOW_H


#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "recordpath.hpp"
//#include "customtaskdialog.h"
#include <QFileInfo>
#include<ctime>

namespace av_console {

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
    void showMessgeInStatusBar(const QString& msg, bool warnning=false);

public Q_SLOTS:
	void on_actionAbout_triggered();
	void on_checkbox_use_environment_stateChanged(int state);
    void on_pushButton_connect_clicked();
    void updateLoggingView(); // no idea why this can't connect automatically
    void updatePathPlanningLoggingView();
    void sensorStatusChanged(int,bool);
    void closeEvent(QCloseEvent *event);

private Q_SLOTS:
    void on_pushButton_gps_clicked(bool checked);
    void on_pushButton_rtk_clicked(bool checked);
    void on_pushButton_pathPlanning_clicked(bool checked);
    void on_pushButton_openRoadNet_clicked();
    void on_pushButton_driverlessStart_clicked(bool checked);
    void on_tabWidget_currentChanged(int index);
    void on_comboBox_taskType_activated(const QString &arg1);
    void onTaskStateChanged(int state);
    void onRosmasterOffline();



private:
    Ui::MainWindow ui;
    QNode qnode;
    RecordPath *m_pathRecorder;
    bool m_nodeInited;
    QString m_pathFileDir;
    //CustomTaskDialog* m_customDialog;

};

}  // namespace av_console

#endif // av_console_MAIN_WINDOW_H
