
#include <QtGui>
#include <QApplication>
#include "../include/main_window.hpp"

int main(int argc, char **argv)
{

    QApplication app(argc, argv);
    av_console::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
