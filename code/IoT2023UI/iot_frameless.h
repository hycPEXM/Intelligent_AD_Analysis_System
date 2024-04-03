#ifndef IOT_FRAMELESS_H
#define IOT_FRAMELESS_H

#include <QWidget>
#include "iotui.h"
#include "framelesswindow.h"

class Iot_frameless : public QWidget
{
    Q_OBJECT
public:
    explicit Iot_frameless(QWidget *parent = nullptr);
    ~Iot_frameless();
    void ShowFrameless();
    IotUI* getLogin();

public slots:
    void CloseFrameless();

private:
    IotUI * m_iot_ui = nullptr;
    FramelessWindow *m_frameless_win = nullptr;

signals:

};

#endif // IOT_FRAMELESS_H
