#include "adpage.h"
#include "imageviewer.h"
#include <QFormLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include "interface.h"

ADPage::ADPage(QWidget* parent)
    : QWidget{ parent }, m_layout(new QVBoxLayout(this)), m_flag_if_start(false)
{
    InitStyle();
    m_layout->setAlignment(Qt::AlignHCenter);
    ImageViewer *viewer = new ImageViewer();
    viewer->setFixedSize(1120,560);
    m_layout->addWidget(viewer);
    m_layout->setSpacing(5);


    QHBoxLayout * form_hlay = new QHBoxLayout();
    form_hlay->setContentsMargins(260, 20, 0,0);
    form_hlay->setAlignment(Qt::AlignHCenter);
    m_layout->addLayout(form_hlay);

    QFormLayout *form = new QFormLayout();
    form->setAlignment(Qt::AlignCenter);
    QLineEdit*lined = new QLineEdit();
    lined->setMinimumHeight(60);
    lined->setMaximumWidth(300);
    lined->setAlignment(Qt::AlignCenter);
    form_hlay->addLayout(form);
//    QPalette palette = lined->palette();
//    palette.setColor(QPalette::Normal,QPalette::PlaceholderText,QColor(128,128,128));
//    lined->setPalette(palette);
    lined->setPlaceholderText("In Seconds");

    form->addRow("Advertising Schedule",lined);
    form->setHorizontalSpacing(20);
    // m_layout->addLayout(form);

    QGridLayout *vlay  = new QGridLayout();
    vlay->setContentsMargins(0, 0, 0, 30);
    vlay->setAlignment(Qt::AlignCenter);
    //vlay->setContentsMargins(450,0,10,0);
    vlay->setSpacing(20);

    QPushButton * but = new QPushButton("Start Push AD");
    but->setObjectName("funcbut");
    but->setFixedSize(230, 70);
    vlay->addWidget(but,0,0);

    connect(but,&QPushButton::clicked,this,[viewer,lined](){
        if(lined->text().isEmpty())
            return;
        saveDir = preDir + "AD_" + QString::number(viewer->getCurrentIndex()+1) +
                "/";
        sendMsg("Start:"+QString::number(viewer->getCurrentIndex() + 1)
                +":" + lined->text(),"192.168.200.200",4000);
    });

    but = new QPushButton("Stop Push AD");
    but->setObjectName("funcbut");
    but->setFixedSize(230, 70);
    vlay->addWidget(but,0,1);
    connect(but,&QPushButton::clicked,this,[](){
        sendMsg("Stop","192.168.200.200",4000);
    });

    but = new QPushButton("Start Analysis");
    but->setObjectName("funcbut");
    but->setFixedSize(230, 70);
    vlay->addWidget(but,1,0);
    connect(but,&QPushButton::clicked,this,[this](){
        if (this->m_flag_if_start) return;
        UpdateChartDir();
        sendMsg("start_yolov2");
        m_flag_if_start = true;
        flag_if_start = true;
    });

    but = new QPushButton("Stop Analysis");
    but->setObjectName("funcbut");
    but->setFixedSize(230,70);
    vlay->addWidget(but,1,1);
    connect(but,&QPushButton::clicked,this,[this](){
        if (!this->m_flag_if_start) return;
        sendMsg("stop_yolov2");
        EndSaveChart();
        UpdateHistoryPage();
        m_flag_if_start = false;
        flag_if_start = false;
    });

    m_layout->addLayout(vlay);
    //vlay->addWidget(but);

    viewer->setImages({ ":/Advertisement/Advertisement/1.JPG",
    ":/Advertisement/Advertisement/2.JPG",
    ":/Advertisement/Advertisement/3.JPG",
    ":/Advertisement/Advertisement/4.JPG",
    ":/Advertisement/Advertisement/5.JPG"
        });
  
}


void ADPage::InitStyle(){
    this->setStyleSheet("QLineEdit{"
                        "color: #C0C0C0;"
                        "background-color:#405361;"
                        "font-size:25px;"
                        "border-style:outset;"
                        "border-radius:10px;"
                        "font-style:MingLiU-ExtB;"
                        "}"
                        // "QLineEdit::placeholder{ color: red; }"
                        "QLabel{"
                        "background:rgba(85,170,255,0);"
                        "color:white;"
                        "font-style:MingLiU-ExtB;"
                        "font-size:30px;"
                        "}"
                        "QPushButton#funcbut{"
                        "background:#ced1d8;"
                        "border-style:outset;"
                        "border-radius:15px;"
                        "font-style:MingLiU-ExtB;"
                        "font-size:18px;"
                        "}"
                        "QPushButton#funcbut:hover{"
                        "background-color:#F08080;"
                        "border-style:inset;"
                        "font-size:18px;"
                        "font-style:MingLiU-ExtB;"
                        "}"
                        );
}
