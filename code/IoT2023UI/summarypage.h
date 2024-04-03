#ifndef SUMMARYPAGE_H
#define SUMMARYPAGE_H

#include <QWidget>
#include <QVBoxLayout>
#include <QButtonGroup>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMap>

class SummaryPage : public QWidget
{
    Q_OBJECT
public:
    explicit SummaryPage(QWidget *parent = nullptr);

private:
    void InitStyle();
    std::vector<int> getRandomList(int count);
    void DrawPipe(const QStringList& order);

private:
    QVBoxLayout *m_layout;
    QButtonGroup m_buttonGroup;
    QGraphicsScene m_scene;
    QGraphicsView m_view;
    QMap<QString,QString> m_ad_color_map;
};

#endif // SUMMARYPAGE_H
