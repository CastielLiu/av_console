
#include <QtGui>
#include <QApplication>
#include "../include/main_window.hpp"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    QSplashScreen splash_screen;
    splash_screen.setPixmap(QPixmap(":/av_console/av_console/splash_screen.jpg"));
    splash_screen.show();
    app.processEvents();

    av_console::MainWindow w(argc,argv,&splash_screen);
    w.show();
    splash_screen.finish(&w);

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    int result = app.exec();

    //app->exit(777);
    if(result == 777) //restart
        QProcess::startDetached(qApp->applicationFilePath(), QStringList());

	return result;
}
