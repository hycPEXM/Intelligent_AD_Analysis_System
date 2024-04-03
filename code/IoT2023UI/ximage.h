#ifndef XIMAGE_H
#define XIMAGE_H

#include <QWidget>
#include <QMenu>
#include <QTimer>


class XImage : public QWidget
{
    Q_OBJECT
public:
    explicit XImage(QWidget *parent = nullptr);
    virtual ~XImage();
    virtual void paintEvent(QPaintEvent *ev);
    virtual void mousePressEvent(QMouseEvent *ev);
    virtual void mouseDoubleClickEvent(QMouseEvent *event);
    void CreateMenu();

signals:

public slots:
    virtual void OpenVideo()  = 0;
    virtual void CloseVideo() = 0;
    virtual void PauseVideo() = 0;
    virtual void ContinueVideo() = 0;
    virtual void CapturePicture() = 0;
    virtual void SaveVideo(bool flag) = 0;
    virtual void updateImg()  = 0;
    virtual void MenuAction(QAction*);


protected:
    QImage src;
    QMenu *m_menu = nullptr;
    QTimer *m_timer;

private:
    int m_x;
    int m_y;
    int m_width;
    int m_height;

};

#endif // XIMAGE_H
