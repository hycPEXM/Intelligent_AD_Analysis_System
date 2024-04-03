#ifndef CHARTSPAGE_H
#define CHARTSPAGE_H

#include <QWidget>
#include <QVBoxLayout>
#include "linechart.h"
#include <QPushButton>
#include <QStackedLayout>

class ChartsPage : public QWidget
{
    Q_OBJECT
public:
    explicit ChartsPage(QWidget *parent = nullptr);

    void addPoint(const QString& name,qreal value);
    void ChangeSaveFile(const QString& date);
    void EndSaveChart();

    virtual ~ChartsPage();

private slots:
    void showPreviousChart();

    void showNextChart();

private:
    void showImage(int index);

    void updateButtons();

    void InitStyle();

private:
    QVBoxLayout *m_layout;
    QHBoxLayout *m_chart_layout;
    QStackedLayout *m_stackedLayout;
    QMap<QString,QPair<long,LineChart*>> m_Charts;
    QPushButton *previousButton;
    QPushButton *nextButton;
    int m_current_index;
};

#endif // CHARTSPAGE_H
