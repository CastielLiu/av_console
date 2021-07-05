#ifndef SPEEDDIAL_H
#define SPEEDDIAL_H

/*
 * 显示速度表盘的自定义控件
 * 使用方式：将Qwidget提升为SpeedDial，然后调用UpdateAngle槽函数更新指针
 * 如要修改样式刻度，参照cpp文件改
 */
#include <QWidget>
#include <QTime>
#include <QTimer>
#include <QPainter>

class SpeedDial : public QWidget
{
    Q_OBJECT
public:
    explicit SpeedDial(QWidget *parent = nullptr);
    void config(int scaleMajor, int scaleMinor, int minVal, int maxVal, float initVal,
                const QString& unit, const QString& title, int numPrecision=1,
                int startAngle=60, int endAngle=-60);

protected:
    void paintEvent(QPaintEvent *);

    void drawCrown(QPainter *painter);
    void drawBackground(QPainter *painter);
    void drawScale(QPainter *painter);
    void drawScaleNum(QPainter *painter);
    void drawTitle(QPainter *painter);
    void drawIndicator(QPainter *painter);
    void drawNumericValue(QPainter *painter);

private:
    QColor m_background;  //背景色
    QColor m_foreground;  //前景色
    QColor m_titleColor;  //标题颜色


    int m_maxValue;
    int m_minValue;
    int m_startAngle;
    int m_endAngle;

    int m_scaleMajor;
    int m_scaleMinor;
    double m_value;
    int m_precision;

    QTimer *m_updateTimer;
    QString m_units;
    QString m_title;

public Q_SLOTS:
    void updateValue(double speed = 0);

};

#endif // SPEEDDIAL_H
