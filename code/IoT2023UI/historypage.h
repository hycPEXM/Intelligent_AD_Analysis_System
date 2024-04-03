#ifndef HISTORYPAGE_H
#define HISTORYPAGE_H

#include <QWidget>
#include <QListView>
#include <QStandardItemModel>
#include <QHBoxLayout>
#include <QStyledItemDelegate>
#include <QStackedLayout>

class CenteredItemDelegate : public QStyledItemDelegate
{
public:
    CenteredItemDelegate(QObject* parent = nullptr) : QStyledItemDelegate(parent) {}

    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override
    {
        QStyleOptionViewItem centeredOption = option;
        centeredOption.displayAlignment = Qt::AlignCenter;

        // 调整上下边距
        int topMargin = 10;
        int bottomMargin = 10;
        centeredOption.rect.adjust(0, -topMargin, 0, bottomMargin);

        QStyledItemDelegate::paint(painter, centeredOption, index);
    }
};


class HistoryPage : public QWidget
{
    Q_OBJECT
public:
    explicit HistoryPage(QWidget *parent = nullptr);
    void UpdateList(const QString& folderPath);
    virtual ~HistoryPage(){
        qDeleteAll(m_model_vector);
        m_model_vector.clear();
    }

private:
//    QStringList getSubfolders(const QString& folderPath);
    void InitStyle();
    void InitListView();

private:
    QVBoxLayout *m_vlay;
    QHBoxLayout *m_hlay;
    QStackedLayout *m_stackedLayout;
    QVector<QStandardItemModel*>  m_model_vector;

};

#endif // HISTORYPAGE_H
