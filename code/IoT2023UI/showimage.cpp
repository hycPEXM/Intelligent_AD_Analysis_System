#include "showimage.h"
#include <QDebug>
#include <QDateTime>
#include <opencv2/videoio.hpp>
#include "interface.h"

static QAction *findActionByName(QMenu *menu, const QString &name)
{
    QList<QAction*> actions = menu->actions();
    for(int i=0; i<actions.size(); i++) {
        if(actions[i]->text() == name) {
            return actions[i];
        }
    }
    return nullptr;
}

ShowImage::ShowImage(QWidget *parent):
    XImage(parent),m_flag_if_play(false),m_flag_if_savevideo(false),
    m_flag_if_cappic(false),m_flag_if_open(false)
{

}

bool ShowImage::InitCamera(){
    m_image = new RTSPImage("rtsp://0.0.0.0:8554/cam");
    if(m_image->OpenCam()){
        return true;
    }
    else
        return false;
}

void ShowImage::OpenVideo(){
    if(m_flag_if_open)  return;
    if(!InitCamera()){
        if (m_image){
            delete  m_image;
            m_image = nullptr;
        }
        return;
    }
    else{
        m_image->start();
    }
    m_flag_if_play = true;
    m_flag_if_open = true;
    m_timer->start();
}

void ShowImage::ContinueVideo(){
    if(m_flag_if_play) return;
    if(m_image)
        m_image->Continue();
    m_timer->start();
}

void ShowImage::PauseVideo(){
    if(!m_flag_if_play) return;
    if(m_image)
        m_image->Pause();
    m_timer->stop();
}

void ShowImage::CapturePicture(){
    if(!m_flag_if_cappic)
        m_flag_if_cappic = true;
}

void ShowImage::SaveVideo(bool flag){
    if(flag){
        QDateTime current = QDateTime::currentDateTime();
        QString video_name = "../Video/"+current.toString("yyMMdd_hhmmss")+".mp4";
        m_flag_if_savevideo = m_writer.open(video_name.toStdString(),cv::VideoWriter::fourcc('M', 'P', '4', 'V')
                      ,16,cv::Size(1920,1080),true);
        if( m_flag_if_savevideo){
            m_SavingVideo_frame = 0;
        }
        else{
            QAction *act = findActionByName(m_menu,"VideoSave");
            act->setChecked(false);
        }
    }
    else{
        if(m_writer.isOpened()){
            m_writer.release();
            m_SavingVideo_frame = 0;
            m_flag_if_savevideo = false;
        }
    }
}

void ShowImage::CloseVideo(){
    if(!m_flag_if_open)  return;
    if(m_flag_if_savevideo){
        ShowImage::SaveVideo(false);
    }
    src = QImage(":/Image/Img/close.png");
    update();
    m_image->requestInterruption();
    m_image->wait();
    if (m_image){
        delete  m_image;
        m_image = nullptr;
    }
    m_flag_if_play    = false;
    m_flag_if_open    = false;
    m_timer->stop();
}

void ShowImage::updateImg(){
    // update();

    if(m_image->GetFrame().empty()) return;
    const cv::Mat &img = m_image->GetFrame();

    if(m_flag_if_cappic){
        QDateTime current = QDateTime::currentDateTime();
        QString img_name = "../Picture/"+current.toString("yyMMdd_hhmmss_zzz")+".png";
        cv::imwrite(img_name.toStdString(),img);
        m_flag_if_cappic = false;
    }

    if(m_flag_if_savevideo){
        cv::Mat tmp;
        cv::resize(img,tmp,cv::Size(1920,1080));
        cvtColor(tmp,tmp,cv::COLOR_RGB2BGR);
        m_writer.write(tmp);
    }


    QImage image = QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
    src = image.scaled(QSize(this->width(), this->height()), Qt::KeepAspectRatio);
    repaint();
}

ShowImage::~ShowImage(){
    ShowImage::CloseVideo();
    if(m_image)
        delete  m_image;
}
