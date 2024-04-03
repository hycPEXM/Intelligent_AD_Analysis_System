#ifndef SETTINGPAGE_H
#define SETTINGPAGE_H

#include <QWidget>
#include <QBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QString>
#include <QPainter>
#include <QPainterPath>


class CircleLabel : public QLabel
{
public:
    CircleLabel(QWidget *parent = nullptr,const QString &path = "")
        : QLabel(parent),m_imgPath(path){}

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QLabel::paintEvent(event);

        // 获取头像图片
        QPixmap pixmap = QPixmap(m_imgPath);
        if (pixmap.isNull())
            return;

        // 绘制圆形头像
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
        QPainterPath path;
        path.addEllipse(QRectF(0, 0, width(), height()));
        painter.setClipPath(path);
        painter.drawPixmap(0, 0, width(), height(), pixmap.scaled(width(), height(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

private:
    QString m_imgPath;
};

class SettingPage : public QWidget
{
    Q_OBJECT
public:
    explicit SettingPage(QWidget *parent = nullptr);

private:
    void InitStyle();

private:
    QVBoxLayout *m_vlay;
    QLineEdit *m_ipLineEdit,*m_portLineEdit,*m_localportLineEdit;
};

#endif // SETTINGPAGE_H
