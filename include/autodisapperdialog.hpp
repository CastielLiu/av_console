#ifndef AUTODISAPPERDIALOG_H
#define AUTODISAPPERDIALOG_H

#include <QObject>
#include <QMessageBox>
#include <QTimer>

/*@brief 可自动消失的提示对话框，当设定的时间内用户没有操作时，页面自动消失
 * 必须使用new动态分配内存，内存由超时系统自动回收，禁止用户手动释放
 * AutoDisapperDialog* dia = new AutoDisapperDialog(this, icon, msg, ms);
 * 当延时参数小于等于0时表示不自动消失
 */
class AutoDisapperDialog : public QMessageBox
{
    Q_OBJECT
public:
    AutoDisapperDialog(QWidget *parent, QMessageBox::Icon icon, const QString& msg, int time_ms)
    {
        mBox = new QMessageBox(icon, "Info", msg,QMessageBox::Yes, parent);
        mBox->show();

        if(time_ms > 0) //延时大于0,开启定时器自动消息
        {
            mTimer = new QTimer(parent);
            mTimer->setSingleShot(true);
            connect(mTimer, SIGNAL(timeout()),this,SLOT(disapper()));
            mTimer->start(time_ms);
        }
        else //阻塞显示，退出后释放内存
        {
            mBox->exec();
            delete mBox;
        }
    }

private Q_SLOTS:
    void disapper()
    {
        mBox->close();
        delete mBox;
        delete mTimer;
        delete this;
    }

private: 
    QTimer *mTimer;
    QMessageBox *mBox;
};

#endif // AUTODISAPPERDIALOG_H
