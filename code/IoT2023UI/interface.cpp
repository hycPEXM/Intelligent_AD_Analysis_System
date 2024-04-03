#include "interface.h"
#include "UDPSocket.h"
#include <QDebug>
#include <QDir>
#include <QDateTime>

static QString Ip_Addr = "";
static uint16_t Ip_port = 0;

static UDPSocket UdpSocket("127.0.0.1");

static ChartsPage* chart_ = nullptr;
static HistoryPage* history_ = nullptr;

QString preDir("/../saveData/");

QString saveDir("/../saveData/AD_1/");


bool flag_if_start = false;

void setChart(ChartsPage *chart){
    chart_ = chart;
}

void setHistory(HistoryPage* history) {
    history_ = history;
}


void UpdateChartDir() {
    if (!chart_) return;
    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString formattedDateTime = currentDateTime.toString("yyMMdd_hhmmss");
    QString pre_dir = QCoreApplication::applicationDirPath() + saveDir + formattedDateTime;
    QDir dir;
    if (dir.mkdir(pre_dir)) {
        qDebug() << "Folder " << pre_dir << " created successfully";
    }
    else {
        qDebug() << "Failed to create folder";
    }
    chart_->ChangeSaveFile(pre_dir);
}

void EndSaveChart() {
    if (!chart_) return;
    chart_->EndSaveChart();
}

void UpdateHistoryPage() {
    history_->UpdateList(saveDir);
}

void InitUdpSocket(uint16_t port) {
    UdpSocket.InitUdp(port);
}

void DeInitUdpSocket() {
    UdpSocket.DeInitUdp();
}

static void RecvProcess(const std::string& msg) {
    if(!flag_if_start) return;
    QString qmsg = QString::fromStdString(msg).trimmed();
    // qDebug() << qmsg;
    QStringList msg_list = qmsg.split(",", Qt::SkipEmptyParts);
    if (msg_list.isEmpty() || msg_list.size() < 0)
        return;
    for (auto& item : msg_list) {
        QStringList item_list = item.split(":", Qt::SkipEmptyParts);
        if (item_list.size() == 2) {
            QString key = item_list[0];
            QString value = item_list[1];
            bool conversionOk = false;
            double numericValue = value.toDouble(&conversionOk);
            if (conversionOk) {
                try {
                    chart_->addPoint(key, numericValue);
                }
                catch (...) {
                    qDebug() << "addPoint Error";
                }
            }
            else {
                qDebug() << "Convert to Double Error";
            }
        }
    }
}

void InitUdpFunction(){
    UdpSocket.setFunction(RecvProcess);
}

bool sendMsg(const QString & msg,const QString& ip,const uint16_t port){
    if (ip.isEmpty() && msg.isEmpty())
        return false;
    return UdpSocket.sendTo(UdpSocket.getSokcetfd(),ip.toStdString()
                             ,port,msg.toStdString());
}

bool sendMsg(const QString & msg){
    if (Ip_Addr.isEmpty() && msg.isEmpty())
        return false;
    return UdpSocket.sendTo(UdpSocket.getSokcetfd(),Ip_Addr.toStdString()
                             ,Ip_port,msg.toStdString());
}

void setSendAddr(const QString& ip, uint16_t port){
    Ip_Addr = ip;
    Ip_port = port;
}

QStringList getSubfolders(const QString& folderPath)
{
    QDir folder(folderPath);
    QStringList subfolders;

    QFileInfoList folderList = folder.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

    for (const QFileInfo& folderInfo : folderList)
    {
        subfolders.append(folderInfo.fileName());
    }

    return subfolders;
}

