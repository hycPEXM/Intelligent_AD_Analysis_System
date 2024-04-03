#include "linechart.h"
#include <QVBoxLayout>
#include <QTextStream>


LineChart::LineChart(QWidget* parent, qreal xMax, qreal yMax,
    const QString& xName, const QString& yName,
    const QString& title, const QString& saveFile)
    : QWidget{ parent }, m_chart(new QChart()), m_series(new QLineSeries()),
    m_axisX(new QValueAxis), m_axisY(new QValueAxis()), m_chartView(new QChartView(m_chart, this)),
    m_timer(this), m_filename(saveFile), m_save_points(0), m_xMax(xMax),m_yCurrMax(yMax),m_yMax(yMax), m_num_iter(0)
{

    m_chart->setTitle(title);
    m_chart->addSeries(m_series);
    // 设置坐标轴名称
    m_axisX->setTitleText(xName);
    m_axisY->setTitleText(yName);

    m_axisX->setTitleFont(QFont("Times New Roman", 15));
    m_axisY->setTitleFont(QFont("Times New Roman", 15));
    m_axisX->setLabelsFont(QFont("Arial", 13));
    m_axisY->setLabelsFont(QFont("Arial", 13));

    // 设置坐标轴刻度范围
    m_axisX->setRange(0, xMax);
    m_axisY->setRange(0, yMax);

    // 添加坐标轴到折线图
    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);

    m_chart->setTheme(QChart::ChartThemeLight);
    m_chart->setTitleFont(QFont("Arial", 25, QFont::Bold));

    m_chart->legend()->setVisible(true);
    m_chart->legend()->setAlignment(Qt::AlignRight);//底部对齐
    m_chart->legend()->setBackgroundVisible(true);//设置背景是否可视
    m_chart->legend()->setAutoFillBackground(true);//设置背景自动填充
    m_chart->legend()->setColor(QColor(222, 233, 251)); //设置颜色
    m_chart->legend()->setLabelColor(QColor(0, 100, 255)); //设置标签颜色
    m_chart->legend()->setMaximumHeight(50);
    m_chart->legend()->setFont(QFont("Arial", 15));

    // 将折线系列关联到坐标轴
    m_series->attachAxis(m_axisX);
    m_series->attachAxis(m_axisY);
    m_series->setName(title);

    // 创建折线图视图,抗锯齿
    m_chartView->setRenderHint(QPainter::Antialiasing);

    // 将折线图视图设置为该QWidget的子部件
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(m_chartView);
    setLayout(layout);

    m_save_points.clear();
    connect(&m_timer, &QTimer::timeout, this, &LineChart::saveData);
    m_timer.setInterval(20 * 1000);
    m_timer.start();
}


// 添加点到折线图
void LineChart::addPoint(qreal x, qreal y)
{
    if (y > m_yCurrMax){
        m_yCurrMax = m_yCurrMax * 1.5;
        m_axisY->setRange(0,m_yCurrMax);
    }
    if (m_series->count() >= m_xMax)
    {
        ++m_num_iter;
        m_save_points += m_series->pointsVector();
        m_series->clear();
        m_axisX->setRange(m_num_iter * m_xMax, m_xMax * (m_num_iter + 1));
    }
    m_series->append(x, y);
}


void LineChart::saveData()
{
    if (m_filename.isEmpty() || m_save_points.empty())
        return;
    QFile file(m_filename);
    if (!file.open(QIODevice::Append | QIODevice::Text))
        return;

    QTextStream out(&file);

    for (const QPointF& point : m_save_points) {
        out << point.x() << " " << point.y() << "\n";
    }
    m_save_points.clear();
    file.close();
}

void LineChart::restoreData(const QString& name) {
    if (name.isEmpty())
        return;

    QFile file(name);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    QTextStream in(&file);
    QVector<QPointF> points;
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList values = line.split(" ");
        if (values.size() != 2)
            continue;

        bool ok;
        qreal x = values[0].toDouble(&ok);
        if (!ok)
            continue;

        qreal y = values[1].toDouble(&ok);
        if (!ok)
            continue;

        points.append(QPointF(x, y));
    }

    file.close();

    auto max_point = std::max_element(points.constBegin(), points.constEnd()
        , [](const QPointF& point1, const QPointF& point2) {
            return point1.y() < point2.y();
        });


    m_series->replace(points);
    setMaxXY(points.size(), max_point->y());
}

LineChart::~LineChart() {
    EndSavePoint();
}


