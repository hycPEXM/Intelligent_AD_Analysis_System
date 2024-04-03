#include "historypage.h"
#include <QDir>
#include <QCoreApplication>
#include <QDebug>
#include <QLabel>
#include "listchart.h"
#include "interface.h"

static QString current_dir;

HistoryPage::HistoryPage(QWidget *parent)
    : QWidget{parent},m_vlay(new QVBoxLayout(this)),m_hlay(new QHBoxLayout())
    ,m_stackedLayout(new QStackedLayout()),m_model_vector(2)
{

    this->setLayout(m_vlay);

    m_vlay->setSpacing(40);
    QHBoxLayout *title_lay = new QHBoxLayout();
    title_lay->setAlignment(Qt::AlignHCenter);
    QLabel *title = new QLabel("History Interface");
    title_lay->addWidget(title);


    m_vlay->addLayout(title_lay);

    m_hlay->setAlignment(Qt::AlignHCenter);
    // m_hlay->setContentsMargins(210,10,10,0);
    m_hlay->addLayout(m_stackedLayout);
    m_vlay->addLayout(m_hlay);

    InitListView();
    InitStyle();
}


void HistoryPage::InitListView(){

    for (auto &item : m_model_vector){
        item = new QStandardItemModel();
    }

    QString pre_dir = QCoreApplication::applicationDirPath() + preDir;
    QStringList subfolders = getSubfolders(pre_dir);
    m_model_vector[0]->clear();
    for (const QString& subfolder : subfolders)
    {
        QStandardItem* item = new QStandardItem(subfolder);
        m_model_vector[0]->appendRow(item);
    }


    QListView* listview = new QListView();
    listview->setModel(m_model_vector[0]);
    listview->setItemDelegate(new CenteredItemDelegate(listview));
    listview->setEditTriggers(QAbstractItemView::NoEditTriggers);
    listview->setSpacing(20);
    listview->setContentsMargins(50,10,10,50);
    listview->setFont(QFont("Arial", 25, QFont::Bold));
    // 处理点击事件的槽函数
    QObject::connect(listview, &QListView::clicked, [this](const QModelIndex& index)
    {
           auto dir = m_model_vector[0]->data(index).toString();
           current_dir = dir;
           UpdateList(dir);
           QStandardItem* item = new QStandardItem("Back");
           m_model_vector[1]->appendRow(item);
           m_stackedLayout->setCurrentIndex(1);
    });
    m_stackedLayout->addWidget(listview);

    listview = new QListView();
    listview->setModel(m_model_vector[1]);
    listview->setItemDelegate(new CenteredItemDelegate(listview));
    listview->setEditTriggers(QAbstractItemView::NoEditTriggers);
    listview->setSpacing(10);
    listview->setContentsMargins(50,10,10,50);
    m_stackedLayout->addWidget(listview);
    QObject::connect(listview, &QListView::clicked, [this](const QModelIndex& index)
    {
           auto dir = m_model_vector[1]->data(index).toString();
           if(dir == "Back"){
                m_stackedLayout->setCurrentIndex(0);
                return;
           }
           auto item = new ListChart(nullptr,preDir + current_dir + "/" + dir);
           item->setGeometry(200,300,this->width(),this->height());
           item->setWindowTitle(dir);
           item->show();
    });
    m_stackedLayout->setCurrentIndex(0);
}

void HistoryPage::UpdateList(const QString& folderPath) {
    QString pre_dir = QCoreApplication::applicationDirPath() + preDir + folderPath;
    QStringList subfolders = getSubfolders(pre_dir);
    m_model_vector[1]->clear();
    for (const QString& subfolder : subfolders)
    {
        QStandardItem* item = new QStandardItem(subfolder);
        m_model_vector[1]->appendRow(item);
    }
}

//QStringList HistoryPage::getSubfolders(const QString& folderPath)
//{
//    QDir folder(folderPath);
//    QStringList subfolders;

//    QFileInfoList folderList = folder.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

//    for (const QFileInfo& folderInfo : folderList)
//    {
//        subfolders.append(folderInfo.fileName());
//    }

//    return subfolders;
//}

void HistoryPage::InitStyle(){
    this->setStyleSheet(
                        "QLabel{"
                        // "background:rgba(85,170,255,0);"
                        "color:white;"
                        "font-style:MingLiU-ExtB;"
                        "font-size:35px;"
                        "}"
                        "QListView{"
                            "color:#DCDCDC;"
                            "font-style:MingLiU-ExtB;"
                            "font-size:25px;"
                            // "border-top: 1px solid  #C0C0C0;"
                            "text-align: center;"
                        "}"
                        "QListView::item{"
                            "border-top: 2px solid  #A9A9A9;"
//                            "border-bottom: 2px solid  #C0C0C0;"
                        "}"
                        );

//    m_listView->setStyleSheet(
//                        "QListView{"
//                            "color:#DCDCDC;"
//                            "font-style:MingLiU-ExtB;"
//                            "font-size:25px;"
//                            // "border-top: 1px solid  #C0C0C0;"
//                            "text-align: center;"
//                        "}"
//                        "QListView::item{"
//                            "border-top: 2px solid  #A9A9A9;"
////                            "border-bottom: 2px solid  #C0C0C0;"
//                        "}"
//                        );
}
