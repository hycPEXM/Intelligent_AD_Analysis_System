#ifndef IOTUI_H
#define IOTUI_H

#include <QWidget>
#include <QPushButton>
#include <QButtonGroup>
#include "adpage.h"
#include "chartspage.h"
#include <QStackedLayout>

QT_BEGIN_NAMESPACE
namespace Ui { class IotUI; }
QT_END_NAMESPACE

class IotUI : public QWidget
{
    Q_OBJECT

public:
    IotUI(QWidget *parent = nullptr);
    ~IotUI();
    QPushButton* getExitBut();

private:
    void InitUI();
    void InitButton();
    void InitShowWin();

private:
    Ui::IotUI *ui;
    QButtonGroup *m_buttonGroup;
    QStackedLayout *m_stack_lay;

};
#endif // IOTUI_H
