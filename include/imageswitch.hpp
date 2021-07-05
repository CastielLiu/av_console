#ifndef IMAGESWITCH_H
#define IMAGESWITCH_H

#include <QWidget>
#include <QMouseEvent>


/*@brief 此类继承于QWidget,用于切换Widget上的图片显示，可用于状态显示，或作为开关控制
 *       使用时将QWidget提升到此类并进行相关操作
 * 使用方法1: 使用内置资源文件中提供的图案进行0,1控制或者状态显示
 * obj->setChecked(false);  //配置为未选中状态
 * obj->setButtonStyle(ImageSwitch::ButtonStyle_4); //选用4号按钮
 * obj->setClickedDisable(); //禁用点击控制
 * 当启用点击控制setClickedEnable()时，被点击时，将发出checked()信号
 *
 * 使用方法2: 使用自定义资源文件提供图案，并根据设定名进行图像切换
 *
 */

#ifdef quc
#if (QT_VERSION < QT_VERSION_CHECK(5,7,0))
#include <QtDesigner/QDesignerExportWidget>
#else
#include <QtUiPlugin/QDesignerExportWidget>
#endif

class QDESIGNER_WIDGET_EXPORT ImageSwitch : public QWidget
#else
class ImageSwitch : public QWidget
#endif

{
    Q_OBJECT
    Q_ENUMS(ButtonStyle)

    //
    Q_PROPERTY(bool isChecked READ getChecked WRITE setChecked)
    //Q_PROPERTY(ButtonStyle buttonStyle READ getButtonStyle WRITE configButton)

public:
    enum ButtonStyle {
        ButtonStyle_1 = 0,  //开关样式1
        ButtonStyle_2 = 1,  //开关样式2
        ButtonStyle_3 = 2,   //开关样式3
        ButtonStyle_4 = 3,
    };

    explicit ImageSwitch(QWidget *parent = 0);
    void setClickedDisable(){useClick = false;}
    void setClickedEnable() {useClick = true;}

protected:
    void mousePressEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent *event);

private:
    bool isChecked;
    ButtonStyle buttonStyle;

    QString imgOffFile;
    QString imgOnFile;
    QString imgFile;
    bool useClick; //是否使用点击，作为状态灯时禁用
    QString mName;  //名称

public:
    bool getChecked()               const;
    ButtonStyle getButtonStyle()    const;
    std::string getName()const {return mName.toStdString();}
    void configButton(const ImageSwitch::ButtonStyle &buttonStyle, const QString& name="");
    /*
    QSize sizeHint()                const;
    QSize minimumSizeHint()         const;
    */
Q_SIGNALS:
    void checked(bool isChecked);

public Q_SLOTS:
    void setChecked(bool isChecked);
};

#endif // IMAGESWITCH_H
