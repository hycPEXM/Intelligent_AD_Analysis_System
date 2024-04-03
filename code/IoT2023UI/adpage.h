#ifndef ADPAGE_H
#define ADPAGE_H

#include <QWidget>
#include <QVBoxLayout>

class ADPage : public QWidget
{
    Q_OBJECT
public:
    explicit ADPage(QWidget *parent = nullptr);

private:
    void InitStyle();

private:
    QVBoxLayout *m_layout;
    bool m_flag_if_start;
};

#endif // ADPAGE_H
