#include "chartspage.h"
#include <QDebug>
#include <QLabel>
#include <QDateTime>
#include <QDir>

ChartsPage::ChartsPage(QWidget *parent)
    : QWidget{parent},m_layout(new QVBoxLayout(this)),m_chart_layout(new QHBoxLayout()),
      m_stackedLayout(new QStackedLayout()),m_current_index(0)
{
    m_layout->setAlignment(Qt::AlignCenter);

    QHBoxLayout *title_lay = new QHBoxLayout();
    title_lay->setAlignment(Qt::AlignHCenter);
    QLabel *title = new QLabel("Visual Interface");
    title_lay->addWidget(title);
    m_layout->addLayout(title_lay);
    m_layout->addLayout(m_chart_layout);

    m_chart_layout->setContentsMargins(0, 0, 0, 0);

    previousButton = new QPushButton();
    previousButton->setFixedSize(40,40);
    // previousButton->setMinimumSize(50,50);
    previousButton->setEnabled(false);
    m_chart_layout->addWidget(previousButton);

    m_chart_layout->addLayout(m_stackedLayout);

    nextButton = new QPushButton();
    nextButton->setFixedSize(40,40);
    m_chart_layout->addWidget(nextButton);

    connect(previousButton, &QPushButton::clicked, this, &ChartsPage::showPreviousChart);
    connect(nextButton, &QPushButton::clicked, this, &ChartsPage::showNextChart);

    // QDateTime currentDateTime = QDateTime::currentDateTime();
    // QString formattedDateTime = currentDateTime.toString("yyMMdd_hhmmss");
    // QString pre_dir = QCoreApplication::applicationDirPath() + "/../../saveData/"+formattedDateTime;
    // QDir dir;
    // if (dir.mkdir(pre_dir)) {
    //     qDebug() << "Folder " << pre_dir << " created successfully";
    // } else {
    //     qDebug() << "Failed to create folder";
    // }

    LineChart *chart = new LineChart(nullptr,400,10,"Time","Focus Num","Focus");
    m_Charts.insert("Focus",qMakePair(0,chart));
    m_stackedLayout->addWidget(chart);

    chart = new LineChart(nullptr,400,10,"Time","UnFocus Num","UnFocus");
    m_Charts.insert("UnFocus",qMakePair(0,chart));
    m_stackedLayout->addWidget(chart);

    chart = new LineChart(nullptr,400,100,"Time","Total Attention","Total");
    m_Charts.insert("Total",qMakePair(0,chart));
    m_stackedLayout->addWidget(chart);

    chart = new LineChart(nullptr,400,1.1,"Time","Advertising Attractiveness",
                          "Attractiveness");
    m_Charts.insert("Attractiveness",qMakePair(0,chart));
    m_stackedLayout->addWidget(chart);

    m_stackedLayout->setCurrentIndex(0);
    InitStyle();
}


void ChartsPage::addPoint(const QString &name, qreal value){
    auto& item = m_Charts[name];
    LineChart *chart = item.second;
//    item.first = int(item.first) % int(chart->getMaxX());
    long point_num = item.first++;
    chart->addPoint(point_num,value);
}

void ChartsPage::ChangeSaveFile(const QString& date) {

    for (auto& item: m_Charts.keys()) {
        LineChart* chart = m_Charts[item].second;
        chart->setSavefile(date +"/" + item + ".txt");
    }
}

void ChartsPage::EndSaveChart() {
    for (auto& item : m_Charts) {
        LineChart* chart = item.second;
        chart->EndSavePoint();
        auto& num = item.first;
        num = 0;
    }
}

void ChartsPage::showPreviousChart(){
    --m_current_index;
    showImage(m_current_index);

    updateButtons();
}

void ChartsPage::showNextChart(){
    ++m_current_index;
    showImage(m_current_index);

    updateButtons();
}

void ChartsPage::showImage(int index) {
    if (index >= 0 && index < m_stackedLayout->count()) {
        m_stackedLayout->setCurrentIndex(index);

    }
}

void ChartsPage::updateButtons() {
    previousButton->setEnabled(m_current_index > 0);
    nextButton->setEnabled(m_current_index < m_stackedLayout->count() - 1);
}

void ChartsPage::InitStyle(){
    this->setStyleSheet(
                        "QLabel{"
                        // "background:rgba(85,170,255,0);"
                        "color:white;"
                        "font-style:MingLiU-ExtB;"
                        "font-size:35px;"
                        "}"
                        );
    previousButton->setStyleSheet("QPushButton{"
                        "border-image: url(:/Image/Img/xiangzuo.png);"
                        "}"
                        "QPushButton:disabled {"
                        "    border-image: url(:/Image/Img/xiangzuo_diable.png);"
                        "}");
    nextButton->setStyleSheet("QPushButton{"
                        "    border-image: url(:/Image/Img/xiangyou.png);"
                        "}"
                        "QPushButton:disabled {"
                        "    border-image: url(:/Image/Img/xiangyou_diable.png);"
                        "}");
}

ChartsPage::~ChartsPage(){
    for (auto& item : m_Charts) {
        delete  item.second;
        item.second = nullptr;
    }
}
