#pragma execution_character_set("utf-8")

#include "imageswitch.hpp"
#include "qpainter.h"
#include "qdebug.h"

ImageSwitch::ImageSwitch(QWidget *parent) :
    QWidget(parent),
    isChecked(false),
    useClick(true)
{
    configButton(ButtonStyle_4);
    setMinimumSize(20,20);
}

void ImageSwitch::mousePressEvent(QMouseEvent *event)
{
    //qDebug() << "mousePressEvent " << useClick << "\t" << event->button();
    if(!useClick)
    {
        QWidget::mousePressEvent(event);
        return;
    }

    if(event->button() == Qt::LeftButton)
    {
        qDebug() << "clicked!";
        isChecked = !isChecked;
        imgFile = isChecked ? imgOnFile : imgOffFile;

        this->update();
        Q_EMIT this->checked(isChecked);
    }
    this->QWidget::mousePressEvent(event);
}

void ImageSwitch::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHints(QPainter::SmoothPixmapTransform);
    QImage img(imgFile);
    //等比、平滑缩放
    //将图片尺寸缩放到小于控件尺寸，防止绘制不全
    img = img.scaled(this->size()*0.9, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    //按照比例自动居中绘制
    int pixX = rect().center().x() - img.width() / 2;
    int pixY = rect().center().y() - img.height() / 2;
    QPoint point(pixX, pixY);
    painter.drawImage(point, img);
    this->QWidget::paintEvent(event);
}

bool ImageSwitch::getChecked() const
{
    return isChecked;
}

ImageSwitch::ButtonStyle ImageSwitch::getButtonStyle() const
{
    return this->buttonStyle;
}

/*
QSize ImageSwitch::sizeHint() const
{
    return QSize(87, 40);
}

QSize ImageSwitch::minimumSizeHint() const
{
    return QSize(87, 40);
}
*/

//@brief 控制开关状态
void ImageSwitch::setChecked(bool isChecked)
{
    if (this->isChecked != isChecked)
    {
        this->isChecked = isChecked;
        imgFile = isChecked ? imgOnFile : imgOffFile;
        this->update();
    }
}

/*@brief 配置按钮
 *@param buttonStyle 按钮类型
 */
void ImageSwitch::configButton(const ImageSwitch::ButtonStyle &buttonStyle, const QString& name)
{
    mName = name;
    if (this->buttonStyle != buttonStyle)
    {
        this->buttonStyle = buttonStyle;

        if (buttonStyle == ButtonStyle_1) {
            imgOffFile = ":/button/imageswitch_imgs/btncheckoff1.png";
            imgOnFile = ":/button/imageswitch_imgs/btncheckon1.png";
            this->resize(87, 28);
        } else if (buttonStyle == ButtonStyle_2) {
            imgOffFile = ":/button/imageswitch_imgs/btncheckoff2.png";
            imgOnFile = ":/button/imageswitch_imgs/btncheckon2.png";
            this->resize(87, 28);
        } else if (buttonStyle == ButtonStyle_3) {
            imgOffFile = ":/button/imageswitch_imgs/btncheckoff3.png";
            imgOnFile = ":/button/imageswitch_imgs/btncheckon3.png";
            this->resize(96, 38);
        } else if (buttonStyle == ButtonStyle_4) {
            imgOffFile = ":/button/imageswitch_imgs/btncheckoff4";
            imgOnFile = ":/button/imageswitch_imgs/btncheckon4";
            this->resize(30,30);
        }
    }
    imgFile = isChecked ? imgOnFile : imgOffFile;
    setChecked(isChecked);
    this->update();
    updateGeometry();
}
