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

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
  void changeToCmdDir();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

  void updateLoggingView(); // no idea why this can't connect automatically
  void updatePathPlanningLoggingView();

private Q_SLOTS:
    void on_button_roscore_clicked();

    void on_pushButton_gps_clicked(bool checked);

    void on_pushButton_rtk_clicked(bool checked);

    void on_pushButton_pathPlanning_clicked(bool checked);
    void showLog_in_pathPlanning(const QString& msg);

private:
    Ui::MainWindow ui;
    QNode qnode;
    RecordPath *m_pathRecorder;
    bool m_nodeInited;
    QString m_pathFileDir;

};

}  // namespace av_console

#endif // av_console_MAIN_WINDOW_H
