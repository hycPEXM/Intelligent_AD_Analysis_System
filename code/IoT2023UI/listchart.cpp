#include "listchart.h"
#include <QCoreApplication>
#include "linechart.h"
#include "interface.h"
#include <QDebug>

ListChart::ListChart(QWidget *parent,const QString& dir)
    : QWidget{parent},m_stackedLayout(new QStackedLayout()),
      m_previousButton(new QPushButton()),m_nextButton(new QPushButton()),
      m_current_index(0)
{
    QString pre_dir = QCoreApplication::applicationDirPath() +dir;
    qDebug() << pre_dir;
    QHBoxLayout* hlay = new QHBoxLayout(this);
    this->setLayout(hlay);

    hlay->setContentsMargins(0, 0, 0, 0);

    m_previousButton->setFixedSize(40,40);
    // previousButton->setMinimumSize(50,50);
    m_previousButton->setEnabled(false);
    hlay->addWidget(m_previousButton);

    hlay->addLayout(m_stackedLayout);

    m_nextButton->setFixedSize(40,40);
    hlay->addWidget(m_nextButton);

    LineChart *chart = new LineChart(nullptr,400,30,"Time","Focus Num","Focus","");
    m_stackedLayout->addWidget(chart);
    chart->restoreData(pre_dir+"/Focus.txt");

    chart = new LineChart(nullptr,400,30,"Time","UnFocus Num","UnFocus","");
    m_stackedLayout->addWidget(chart);
    chart->restoreData(pre_dir+"/UnFocus.txt");

    chart = new LineChart(nullptr,400,100,"Time","Total Attention","Total","");
    m_stackedLayout->addWidget(chart);
    chart->restoreData(pre_dir+"/Total.txt");

    chart = new LineChart(nullptr,400,1.1,"Time","Advertising Attractiveness",
                          "Attractiveness","");
    chart->restoreData(pre_dir+"/Attractiveness.txt");
    m_stackedLayout->addWidget(chart);

    m_stackedLayout->setCurrentIndex(0);
    InitStyle();

    connect(m_previousButton, &QPushButton::clicked, this, &ListChart::showPreviousChart);
    connect(m_nextButton, &QPushButton::clicked, this, &ListChart::showNextChart);
}

void ListChart::showPreviousChart(){
    --m_current_index;
    showImage(m_current_index);

    updateButtons();
}

void ListChart::showNextChart(){
    ++m_current_index;
    showImage(m_current_index);

    updateButtons();
}

void ListChart::showImage(int index) {
    if (index >= 0 && index < m_stackedLayout->count()) {
        m_stackedLayout->setCurrentIndex(index);

    }
}

void ListChart::updateButtons() {
    m_previousButton->setEnabled(m_current_index > 0);
    m_nextButton->setEnabled(m_current_index < m_stackedLayout->count() - 1);
}

void ListChart::InitStyle(){
    m_previousButton->setStyleSheet("QPushButton{"
                        "border-image: url(:/Image/Img/xiangzuo.png);"
                        "}"
                        "QPushButton:disabled {"
                        "    border-image: url(:/Image/Img/xiangzuo_diable.png);"
                        "}");
    m_nextButton->setStyleSheet("QPushButton{"
                        "    border-image: url(:/Image/Img/xiangyou.png);"
                        "}"
                        "QPushButton:disabled {"
                        "    border-image: url(:/Image/Img/xiangyou_diable.png);"
                        "}");
}
