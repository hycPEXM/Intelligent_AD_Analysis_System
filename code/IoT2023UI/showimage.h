#ifndef SHOWIMAGE_H
#define SHOWIMAGE_H

#include <QMap>
#include "ximage.h"
#include "rtspimage.h"

class ShowImage : public XImage
{
    Q_OBJECT
public:
    ShowImage(QWidget *parent = nullptr);
    ~ShowImage();

public slots:
    virtual void OpenVideo() override;
    virtual void CloseVideo() override;
    virtual void PauseVideo() override;
    virtual void ContinueVideo() override;
    virtual void CapturePicture()   override;
    virtual void SaveVideo(bool flag) override;
    virtual void updateImg() override;

private:
    bool InitCamera();

private:
    bool m_flag_if_play;
    bool m_flag_if_savevideo;
    bool m_flag_if_cappic;
    bool m_flag_if_open;

    cv::VideoWriter m_writer;
    RTSPImage *m_image;

    int m_SavingVideo_frame = 0;
};

#endif // SHOWIMAGE_H
