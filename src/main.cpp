
#include <QtGui>
#include <QApplication>
#include "../include/main_window.hpp"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    av_console::MainWindow w(argc,argv);


    //setWindowFalgs() must before show()
    w.setWindowFlags(w.windowFlags() | Qt::WindowStaysOnTopHint); //set always on the top
    //w.setWindowFlags(w.windowFlags() | Qt::FramelessWindowHint);  //hidden title
    w.setGeometry(63,52, 737,548);
    w.setFixedSize(737,548);

    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    //app->exit(777);
    if(result == 777) //restart
        QProcess::startDetached(qApp->applicationFilePath(), QStringList());

	return result;
}
