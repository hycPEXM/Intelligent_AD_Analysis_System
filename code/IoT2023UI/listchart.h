#ifndef LISTCHART_H
#define LISTCHART_H

#include <QWidget>
#include <QBoxLayout>
#include <QStackedLayout>
#include <QPushButton>


class ListChart : public QWidget
{
    Q_OBJECT
public:
    explicit ListChart(QWidget *parent = nullptr,const QString& dir = "");

private:
    void Destory()
    {
        deleteLater();
    }

private slots:
    void showPreviousChart();

    void showNextChart();

private:
    void showImage(int index);

    void updateButtons();

    void InitStyle();

protected:
    void closeEvent(QCloseEvent *event) override
    {
        Destory();
        QWidget::closeEvent(event);
    }

private:
    QStackedLayout *m_stackedLayout;
    QPushButton *m_previousButton;
    QPushButton *m_nextButton;
    int m_current_index;
};

#endif // LISTCHART_H
