#include "iotui.h"
#include "ui_iotui.h"
#include "historypage.h"
#include <QTimer>
#include <QRandomGenerator>
#include "settingpage.h"
#include "interface.h"
#include "rtsppage.h"
#include "summarypage.h"

IotUI::IotUI(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::IotUI),m_buttonGroup(nullptr)
{
    ui->setupUi(this);
    InitUI();

    connect(ui->pushButton_2,&QPushButton::clicked,this,[this](){
            this->m_stack_lay->setCurrentIndex(0);
    });
    connect(ui->pushButton_3,&QPushButton::clicked,this,[this](){
            this->m_stack_lay->setCurrentIndex(1);
    });
    connect(ui->pushButton_4,&QPushButton::clicked,this,[this](){
            this->m_stack_lay->setCurrentIndex(2);
    });
    connect(ui->pushButton_5,&QPushButton::clicked,this,[this](){
            this->m_stack_lay->setCurrentIndex(4);
    });

    connect(ui->pushButton_6,&QPushButton::clicked,this,[this](){
            this->m_stack_lay->setCurrentIndex(3);
    });

//    int count = 0;
//    static QTimer timer;
//    QObject::connect(&timer, &QTimer::timeout, [this,count]()mutable{
//        qreal y = qreal(count) /3;
//        count = (count +1)%80;
//        auto item =static_cast<ChartsPage*>(this->m_stack_lay->itemAt(1)->widget());
//        item->addPoint("Focus",y);
//    });
//    timer.start(40);

    InitUdpFunction();
}


void IotUI::InitUI(){
    InitButton();
    InitShowWin();
}

void IotUI::InitButton(){
    m_buttonGroup = new QButtonGroup(this);
    m_buttonGroup->setExclusive(true);

    ui->pushButton_2->setCheckable(true);
    ui->pushButton_3->setCheckable(true);
    ui->pushButton_4->setCheckable(true);
    ui->pushButton_5->setCheckable(true);
    ui->pushButton_6->setCheckable(true);

    ui->pushButton_2->setChecked(true);
    m_buttonGroup->addButton(ui->pushButton_2);
    m_buttonGroup->addButton(ui->pushButton_3);
    m_buttonGroup->addButton(ui->pushButton_4);
    m_buttonGroup->addButton(ui->pushButton_5);
    m_buttonGroup->addButton(ui->pushButton_6);
}

void IotUI::InitShowWin(){
    auto win = ui->widget;
    m_stack_lay = new QStackedLayout(win);
    win->setLayout(m_stack_lay);
    auto ad_viewer = new ADPage();
    ad_viewer->setGeometry((win->width()- 1160)/2,(win->height() - 980)/2,1160,980);
    m_stack_lay->addWidget(ad_viewer);

    auto chart_viewer = new ChartsPage();
    setChart(chart_viewer);
    chart_viewer->setGeometry((win->width() - 1160) / 2, (win->height() - 980) / 2, 1160, 980);
    m_stack_lay->addWidget(chart_viewer);

//    auto rtsp_viewer = new RTSPPage();
//    rtsp_viewer->setGeometry((win->width() - 1160) / 2, (win->height() - 980) / 2, 1160, 980);
//    m_stack_lay->addWidget(rtsp_viewer);

    auto history_viewer = new HistoryPage();
    setHistory(history_viewer);
    chart_viewer->setGeometry((win->width() - 1160) / 2, (win->height() - 980) / 2, 1160, 980);
    m_stack_lay->addWidget(history_viewer);

    auto summary_viewer = new SummaryPage();
    summary_viewer->setGeometry((win->width() - 1160) / 2, (win->height() - 980) / 2, 1160, 980);
    m_stack_lay->addWidget(summary_viewer);

    auto setpage = new SettingPage();
    setpage->setGeometry((win->width()-680)/2,(win->height() - 720)/2,680,700);
    m_stack_lay->addWidget(setpage);
    m_stack_lay->setCurrentIndex(0);
    win->show();
}

IotUI::~IotUI()
{
    delete m_buttonGroup;
    delete ui;
}

QPushButton* IotUI::getExitBut(){
     return ui->pushButton;
}

