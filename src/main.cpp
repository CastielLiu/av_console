
#include <QtGui>
#include <QApplication>
#include <QString>
#include <iostream>
#include "../include/main_window.hpp"
#include "utils.hpp"

using namespace av_console;

bool loadRosNodesArrayInfo(const QString& file);

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    QString appDir = QApplication::applicationDirPath();

    //初始化日志输出
    if(!g_logg.init(appDir + "/../log/log.txt"))
        return 0;

    //载入RosNodesArray信息
    if(!loadRosNodesArrayInfo(appDir + "/../cmd/cmd.xml"))
        return 0;

    //程序启动画面
    QSplashScreen splash_screen;
    splash_screen.setPixmap(QPixmap(":/av_console/av_console/splash_screen.jpg"));
    splash_screen.show();
    app.processEvents();

    av_console::MainWindow w(argc,argv,&splash_screen);
#if(DEVICE == DEVICE_LOGISTICS)
    w.setWindowFlags(w.windowFlags() | Qt::WindowStaysOnTopHint); //窗口置顶
    //w.setWindowFlags(w.windowFlags() | Qt::FramelessWindowHint);//隐藏标题
    w.setGeometry(63,52, 737,548);
    w.setFixedSize(737,548);
#endif

    w.show();

    splash_screen.finish(&w);

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    int result = app.exec();

    //app->exit(777);
    if(result == 777) //restart
        QProcess::startDetached(qApp->applicationFilePath(), QStringList());

    return result;
}

bool loadRosNodesArrayInfo(const QString& file)
{
    tinyxml2::XMLDocument Doc;   //定义xml文件对象
    tinyxml2::XMLError res = Doc.LoadFile(file.toStdString().c_str());

    if(tinyxml2::XML_ERROR_FILE_NOT_FOUND == res)
    {
        QString fatalMsg = QString("No ") + file + "!";
        g_logg << fatalMsg << "\n";
        return false;
    }
    else if(tinyxml2::XML_SUCCESS != res)
    {
        QString fatalMsg = QString("Parse ") + file + "failed!";
        g_logg << fatalMsg << "\n";
        return false;
    }
    tinyxml2::XMLElement *pRoot = Doc.RootElement();

    //第一个子节点 Nodes
    const tinyxml2::XMLElement *pRosNodes = pRoot->FirstChildElement();
    int node_id = 0;
    while(pRosNodes)
    {
        RosNodes ros_nodes;
        ros_nodes.name = pRosNodes->Attribute("name");
        if(const char* t = pRosNodes->Attribute("show_status"))
            ros_nodes.show_status = t;

        qDebug() << pRosNodes->Attribute("show_status");
        ros_nodes.use_button = pRosNodes->BoolAttribute("use_botton");
        ros_nodes.id = node_id++;

        const tinyxml2::XMLElement *pChild = pRosNodes->FirstChildElement();
        while(pChild)
        {
            std::string child_name(pChild->Name());
            if(child_name == "LaunchCommand")
            {
                std::string launch_cmd(pChild->GetText());
                ros_nodes.launch_cmd = launch_cmd;
            }
            else if(child_name == "CloseCommand")
            {
                ros_nodes.close_cmd = std::string(pChild->GetText());
            }
            else if(child_name == "Topic")
            {
                std::string topic_name(pChild->Attribute("name"));
                std::string topic_val(pChild->GetText());

                ros_nodes.topics[topic_name] = topic_val;
            }
            pChild = pChild->NextSiblingElement();//转到下一子节点
        }
        g_rosNodesArray[ros_nodes.name] = ros_nodes;
        pRosNodes = pRosNodes->NextSiblingElement();//转到下一子节点，链表结构
    }

    displayRosNodesArrayInfo();
    return true;
}
