
#ifndef av_console_MAIN_WINDOW_H
#define av_console_MAIN_WINDOW_H


#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "recordpath.hpp"
#include "recorddata.hpp"
//#include "customtaskdialog.h"
#include <QFileInfo>
#include <QObjectList>
#include <QCheckBox>
#include "utils.hpp"
#include "globalvariables.hpp"
#include <ctime>
#include <queue>
#include <algorithm>
#include <tinyxml2.h>
#include <QSplashScreen>
#include <unordered_map>
#include <QButtonGroup>
#include "imageswitch.hpp"

#define FontLevel1 16
#define FontLevel2 14
#define FontLevel3 12
#define FontLevel4 10
#define FontLevel5 9

namespace av_console {

class SensorDiagnostic
{
  public:
    typedef struct Sensor
    {
        QString name;
        bool scrol_view;
    }Sensor_t;

private:
};

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QSplashScreen* splash, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void showNoMasterMessage();
    bool changeToCmdDir(bool mode=false);
    void initSensorStatusWidget();
    void showMessgeInStatusBar(const QString& msg, bool warnning=false);

    void killDriverlessNode(){
        system("gnome-terminal -e \"rosnode kill /driverless_node\"");
    }
    void setPushButtonStylesheet(const QString& style);

public Q_SLOTS:
    void on_actionAbout_triggered();
    void on_actionReinstall_triggered();
    void on_actionEditCommandFile_triggered();
    void on_actionHelp_triggered();

	void on_checkbox_use_environment_stateChanged(int state);
    void on_pushButton_connect_clicked();
    void updateLoggingView(); // no idea why this can't connect automatically
    void updatePathPlanningLoggingView();
    void sensorStatusChanged(int,bool);
    void closeEvent(QCloseEvent *event);
    void showEvent(QShowEvent* event) override;
    void onDriverlessStatusChanged(const driverless_common::SystemState state);

Q_SIGNALS:
    void restart();

private Q_SLOTS:
    void on_pushButton_pathPlanning_clicked(bool checked);
    void on_pushButton_openRoadNet_clicked();
    void on_pushButton_driverlessStart_clicked(bool checked);
    void on_comboBox_taskType_activated(const QString &arg1);
    void onTaskStateChanged(int state, const QString &info="");
    void onRosmasterOffline();
    void onTimeout();
    void on_pushButton_startRecordData_clicked(bool checked);
    void on_pushButton_selectRecordFile_clicked();
    void updateDataRecorderLoggingView();
    void onShowDiagnosticMsg(const QString& device, int level, const QString& msg);

    void on_pushButton_setPathWidth_clicked();
    void on_pushButton_setLeftTurn_clicked(bool checked);
    void on_pushButton_setRightTurn_clicked(bool checked);
    void on_pushButton_setParkPoint_clicked();
    void on_pushButton_setRoadMaxSpeed_clicked(bool checked);
    void customizeButtonsClicked(bool checked);
    void on_pushButton_setTrafficLightPoint_clicked();
    void on_treeWidget_diagnosics_itemDoubleClicked(QTreeWidgetItem *item, int column);
    void on_treeWidget_fixedDiagnostic_itemDoubleClicked(QTreeWidgetItem *item, int column);
    void onDiagnosticsWidgetHeaderResize(int idx,int old_w,int new_w);
    void onDiagnosticsWidgetHeaderDoubleClicked(int index);

private:
    void setWidgetItemColorByMsgLevel(QTreeWidgetItem* item, int level);
    QObjectList getAllLeafChilds(QObject* object);
    void disableRecordDataConfigure(bool flag);
    void displayRosNodesArrayInfo();
    bool initDriverlessSystem();
    bool initQnode();

private:
    Ui::MainWindow ui;
    QNode qnode;
    RecordPath *m_pathRecorder;
    RecordData *m_dataRecorder;
    bool m_nodeInited;
    QString m_roadNetFileDir;
    QString m_recordFileDir;

    bool m_forceExit; //强制退出，无需确认
    QTimer mTimer;
    QString mCmdFileName;
    bool m_rosNodesArrayInvalid;
    QSplashScreen *m_splash;

    std::unordered_map<int, ImageSwitch *> m_sensorStatusWidgets; //id,widget

    //诊断信息包含两个窗口，一个滚动窗口，一个实时更新窗口(每个设备仅显示一行)
    QStringList m_realtimeDeviceList; //诊断信息窗口中，需要实时更新的设备列表
    bool m_diagnosticsAntoScroll = true;

    QButtonGroup* m_btnGroupChangeWidget;
};

}  // namespace av_console

#endif // av_console_MAIN_WINDOW_H
