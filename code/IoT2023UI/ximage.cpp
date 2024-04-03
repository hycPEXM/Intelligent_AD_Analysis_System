#include "ximage.h"
#include <QPainter>
#include <QMouseEvent>

#include <QDebug>
#include <QTimer>


XImage::XImage(QWidget *parent)
    : QWidget{parent}
{
    this->setStyleSheet("background-color:WhiteSmoke");
    m_menu = new QMenu(this);
    CreateMenu();
    src = QImage(":Image/Img/close.png");
    m_timer = new QTimer(this);
    m_timer->setInterval(50);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(updateImg()));
}

XImage::~XImage(){
    delete m_timer;
    m_timer = nullptr;
}


void XImage::paintEvent(QPaintEvent *ev)
{
    QPainter p(this);

    //显示图片
    if (!src.isNull())
        p.drawImage((this->width()-src.width())/2, (this->height()-src.height())/2, src);
}

void XImage::mousePressEvent(QMouseEvent *ev){
    if(ev->button()& Qt::RightButton){
        //右键菜单
        QPoint gpos = mapToGlobal(ev->pos());
        gpos.setX(gpos.x()+ 10);
        m_menu->exec(gpos);
    }
}

void XImage::mouseDoubleClickEvent(QMouseEvent *event){
    if(isFullScreen()){
        this->setWindowFlags(Qt::SubWindow);
        this->setGeometry(m_x, m_y, m_width, m_height);//设置位置大小 x、y、w、h
        this->showNormal();
    }
    else{
        m_x = this->x();
        m_y = this->y();
        m_width = this->width();
        m_height = this->height();
        this->setWindowFlags(Qt::Window);
        this->showFullScreen();
    }
    //qDebug() << "mouseDoubleClickEvent "<<event->pos().x()<<":" << event->pos().y();
}

//初始化按钮组
void XImage::CreateMenu(){
    m_menu->addAction("Open");
    m_menu->addAction("Continue");
    m_menu->addAction("Pause");
    m_menu->addAction("Capture");
    QAction *save_act = m_menu->addAction("VideoSave");
    m_menu->addAction("Close");
    QString style =  R"(
                            QMenu{
                                border-radius: 0px;
                            }
                            QMenu::item{
                                 background-color: white;
                            }
                            QMenu::item::selected{
                                 background-color: lightblue;
                            }
                            QMenu::item:checked {
                                 color: Snow;
                                 border: 1px solid Snow;
                                 background-color: LimeGreen;
                            }
                         )";
    this->setStyleSheet(style);
    m_menu->setStyleSheet("QMenu{border: 0px solid #A0A0A0;border-radius:0;}");
    save_act->setCheckable(true);

    connect(m_menu, SIGNAL(triggered(QAction*)), this, SLOT(MenuAction(QAction*)));
}

//设置按钮组的SLOT函数
void XImage::MenuAction(QAction* act){
    if(act->text() == "Open") OpenVideo();
    else if(act->text() == "Close") CloseVideo();
    else if(act->text() == "Pause") PauseVideo();
    else if(act->text() == "Continue") ContinueVideo();
    else if(act->text() == "Capture") CapturePicture();
    else if(act->text() == "VideoSave") SaveVideo(act->isChecked());
    else return;
}
