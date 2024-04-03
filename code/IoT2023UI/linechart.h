#ifndef LINECHART_H
#define LINECHART_H

#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QChartView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QApplication>
#include <QTimer>

QT_CHARTS_USE_NAMESPACE

class LineChart : public QWidget
{
    Q_OBJECT

public:
    explicit LineChart(QWidget* parent = nullptr, qreal xMax = 0, qreal yMax = 0,
        const QString& xName = "x", const QString& yName = "y",
        const QString& title = "title", const QString& saveFile = "");

    virtual ~LineChart();
    // 添加点到折线图
    void addPoint(qreal x, qreal y);
    qreal getMaxX() { return m_xMax; }
    void restoreData(const QString& name);
    void setSavefile(const QString& name) {
        if (name.isEmpty())
            return;
        m_filename = name;
    }
    void EndSavePoint() {
        m_save_points += m_series->pointsVector();
        m_series->clear();
        saveData();
        setMaxXY(m_xMax,m_yMax);
        m_num_iter = 0;
    }

public slots:
    void saveData();

private:
    void setMaxXY(qreal x, qreal y) {
        m_xMax = x;
        m_yCurrMax = y;
        m_axisX->setRange(0, x);
        m_axisY->setRange(0, y);
    }

    void setXYName(const QString& xName, const QString& yName) {
        m_axisX->setTitleText(xName);
        m_axisY->setTitleText(yName);
    }

private:
    QChart* m_chart;
    QLineSeries* m_series;
    QValueAxis* m_axisX;
    QValueAxis* m_axisY;
    QChartView* m_chartView;

    QTimer m_timer;
    QString m_filename;
    QVector<QPointF> m_save_points;
    qreal m_xMax, m_yCurrMax,m_yMax;
    int m_num_iter;
};


#endif // LINECHART_H
