#ifndef INTERFACE_H
#define INTERFACE_H

#include <QString>
#include "chartspage.h"
#include "historypage.h"
extern QString preDir;
extern QString saveDir;
extern bool flag_if_start;

bool sendMsg(const QString & msg,const QString& ip,const uint16_t port);
bool sendMsg(const QString & msg);
void setSendAddr(const QString& ip, uint16_t port);
void setChart(ChartsPage *chart);
void setHistory(HistoryPage* history);
void UpdateChartDir();
QStringList getSubfolders(const QString& folderPath);
void EndSaveChart();
void UpdateHistoryPage();
void InitUdpFunction();
void InitUdpSocket(uint16_t port);
void DeInitUdpSocket();

#endif // INTERFACE_H
