#include "iot_frameless.h"

Iot_frameless::Iot_frameless(QWidget *parent)
    : QWidget{parent}
{
    m_iot_ui = new IotUI();
    m_frameless_win = new FramelessWindow(m_iot_ui);
    QPushButton* temp = m_iot_ui->getExitBut();
    connect(temp,&QPushButton::clicked,this,&Iot_frameless::CloseFrameless);
    // m_frameless_win->resize(m_login->width(),m_login->height());
    m_frameless_win->setGeometry(1000,200,m_iot_ui->width(),m_iot_ui->height());
}


void Iot_frameless::ShowFrameless(){
    m_frameless_win->show();
}

void Iot_frameless::CloseFrameless(){
    m_frameless_win->close();
}

IotUI* Iot_frameless::getLogin(){
    return m_iot_ui;
}

Iot_frameless::~Iot_frameless(){
    delete  m_frameless_win;
}


