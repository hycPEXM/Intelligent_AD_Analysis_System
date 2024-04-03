#include "summarypage.h"
#include <QLabel>
#include <QButtonGroup>
#include <QCheckBox>
#include "interface.h"
#include <algorithm>
#include <random>
#include <QtCore>
#include <QMap>
#include <QStringList>
#include <QVector>

QStringList color_table{ "#F08080" , "#F4A460", "#BDB76B" ,
                         "#98FB98" , "#40E0D0", "#87CEFA" ,
                         "#7B68EE" , "#DDA0DD"};

static QVector<qreal> calAvgAttr(const QVector<qreal>& inputVector, int numGroups){
    QVector<qreal> result;
    int vectorSize = inputVector.size();
    if(vectorSize < 12){
        result = inputVector;
        result.resize(12);
        return result;
    }
    int groupSize = vectorSize / numGroups;

    for (int i = 0; i < numGroups; i++) {
        int startIndex = i * groupSize;
        int endIndex = (i + 1) * groupSize;
        if (i == numGroups - 1) {
            endIndex = vectorSize; // 最后一组可能不够groupSize，将其包含进来
        }

//        double sum = std::accumulate(inputVector.begin() + startIndex
//                                      , inputVector.begin() + endIndex, 0);
        qreal sum = 0;
        for (qreal element : inputVector.mid(startIndex, endIndex - startIndex)) {
            sum += element;
        }
//        for (int j = startIndex; j < endIndex; j++) {
//            sum += inputVector[j];
//        }

        double average = sum / (double)(endIndex - startIndex);
//        qDebug() << average;
        result.append(average);
    }

    return result;
}

static QString findNewestFolder(const QString& path)
{
    QDir dir(path);
    QStringList subDirs = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);

    if (subDirs.isEmpty())
    {
        qWarning() << "No subdirectories found in" << path;
        return QString();
    }

    QString newestFolder = subDirs.first();
    QDateTime newestFolderTime = QFileInfo(dir.filePath(newestFolder)).birthTime();

    foreach (const QString& subDir, subDirs)
    {
        QDateTime subDirTime = QFileInfo(dir.filePath(subDir)).birthTime();
        if (subDirTime > newestFolderTime)
        {
            newestFolder = subDir;
            newestFolderTime = subDirTime;
        }
    }

    return newestFolder;
}


static QVector<qreal> getData(const QString& name) {
    if (name.isEmpty())
        return QVector<qreal>(12);

    QFile file(name);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return QVector<qreal>(12);

    QTextStream in(&file);
    QVector<qreal> points;
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList values = line.split(" ");
        if (values.size() != 2)
            continue;

        bool ok;
        qreal y = values[1].toDouble(&ok);
        if (!ok)
            continue;

        points.append(y);
    }

    file.close();
    return calAvgAttr(points,12);
}

static QStringList getSummaryList(const QString& predir ,const QStringList& ADDirs){

    QMap<QString,QVector<qreal>> map;
    QStringList summary;
    for (auto& ADDir : ADDirs){
        QString lastestDir = findNewestFolder(predir + ADDir);
        auto data = getData(predir + ADDir + "/" + lastestDir + "/Attractiveness.txt");
        // qDebug() << predir + ADDir + "/" + lastestDir  + "/Attractiveness.txt" << ":" << data;
        map.insert(ADDir,data);
    }

    for (int i = 0; i < 12; ++i){
        using SummaryMap = std::pair<QString,QVector<qreal>>;
        auto maxElement = std::max_element(map.constKeyValueBegin(), map.constKeyValueEnd(),
            [i](const SummaryMap& pair1, const SummaryMap& pair2) {
                const auto& vector1 = pair1.second;
                const auto& vector2 = pair2.second;

                if (vector1.isEmpty() || vector2.isEmpty()) {
                    return false; // 忽略空的向量
                } else {
                    return vector1.at(i) < vector2.at(i); // 假设索引为0
                }
            });
        summary.append(maxElement->first);
    }
    return summary;
}

SummaryPage::SummaryPage(QWidget *parent)
    : QWidget{parent},m_layout(new QVBoxLayout(this)),
      m_scene(),m_view(&m_scene),m_ad_color_map()
{

    m_layout->setAlignment(Qt::AlignCenter);
    m_layout->setSpacing(15);

    QHBoxLayout *title_lay = new QHBoxLayout();
    title_lay->setAlignment(Qt::AlignHCenter);
    QLabel *title = new QLabel("Summary");
    title_lay->addWidget(title);
    m_layout->addLayout(title_lay);
  //    m_layout->setAlignment(Qt::AlignCenter);  title_lay->setContentsMargins(0,20,0,20);

    auto &&random_vector = getRandomList(8);
    int index = 0;

    QHBoxLayout *check_lay = new QHBoxLayout();
    check_lay->setAlignment(Qt::AlignHCenter);
    check_lay->setSpacing(40);
    QString pre_dir = QCoreApplication::applicationDirPath() + preDir;
    QStringList subfolders = getSubfolders(pre_dir);
    for ( auto & subfolder : subfolders){
        QCheckBox *box = new QCheckBox(subfolder);
        check_lay->addWidget(box);
        m_buttonGroup.addButton(box);
        m_ad_color_map.insert(subfolder,color_table[random_vector[index]]);
        index ++;
    }


    m_buttonGroup.setExclusive(false);
    m_layout->addLayout(check_lay);


    QHBoxLayout *pipe_lay = new QHBoxLayout();
    m_view.setRenderHint(QPainter::Antialiasing);
    m_view.setWindowTitle(" Time Distribution");
    m_view.setFixedSize(700,700);
    pipe_lay->addWidget(&m_view);
    pipe_lay->setAlignment(Qt::AlignHCenter);
    m_layout->addLayout(pipe_lay);


    QHBoxLayout *button_lay = new QHBoxLayout();
    button_lay->setAlignment(Qt::AlignHCenter);
    QPushButton *but = new QPushButton("AD Strategy Analysis");
    but->setFixedSize(230, 70);
    button_lay->addWidget(but);
    m_layout->addLayout(button_lay);

    connect(but,&QPushButton::clicked,this,[this](){
        m_scene.clear();
        QList<QAbstractButton*> Buttons = m_buttonGroup.buttons();
        QStringList selectedList;
        for (QAbstractButton* button : Buttons) {
            if(button->isChecked()){
                QString buttonText = button->text();
                selectedList.append(buttonText);
            }
        }
        if(selectedList.isEmpty())
        {
            return;
        }
//        qDebug() << selectedList;
        QString pre_dir = QCoreApplication::applicationDirPath() + preDir;
        auto drawList = getSummaryList(pre_dir,selectedList);
        DrawPipe(drawList);
//        DrawPipe({"AD_1","AD_2","AD_5","AD_3","AD_1","AD_2",
//                  "AD_3","AD_5","AD_3","AD_4","AD_2","AD_4"});
    });

    InitStyle();
}

std::vector<int> SummaryPage::getRandomList(int count)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<int> numbers(count);
    for (int i = 0; i < count; ++i) {
        numbers[i] = i;
    }
    std::shuffle(numbers.begin(), numbers.end(),gen);
    return numbers;
}


void SummaryPage::DrawPipe(const QStringList& order){

    if(order.size() != 12) return;
    // 设置圆的半径和中心点
    int radius = 300;
    QPointF center(m_view.width() / 2, m_view.height() / 2);

    // 创建圆的边界点
    QPolygonF circle;
    for (int i = 0; i < 12; ++i) {
        qreal angle = (i - 3) * 30.0 * 3.14159265 / 180.0;
        qreal x = center.x() + radius * qCos(angle);
        qreal y = center.y() + radius * qSin(angle);
        circle << QPointF(x, y);
    }

    // 在场景中绘制圆形
    QGraphicsPolygonItem circleItem(circle);
    circleItem.setBrush(QBrush(Qt::white));
    circleItem.setPen(QPen(Qt::black));
    m_scene.addItem(&circleItem);

    // 在每个刻度位置添加事件标签
    for (int i = 0; i < 12; ++i) {
        qreal angle = (i + 0.5 - 3) * 30.0 * 3.14159265 / 180.0;
        qreal x = center.x() + (radius + 30) * qCos(angle);
        qreal y = center.y() + (radius + 30) * qSin(angle);

        QGraphicsTextItem *eventLabel = new QGraphicsTextItem(order[i]);
        eventLabel->setDefaultTextColor(QColor("#F0FFF0"));
        eventLabel->setFont(QFont("Arial", 15 ,QFont::Bold));
        eventLabel->setPos(QPointF(x - 25, y ));
        m_scene.addItem(eventLabel);

        // 设置刻度颜色
        QPolygonF wedge;
        wedge << center << circle[i] << circle[(i + 1) % 12];
        QGraphicsPolygonItem *wedgeItem = new QGraphicsPolygonItem(wedge);
        wedgeItem->setBrush(QBrush(QColor(m_ad_color_map[order[i]])));
        wedgeItem->setPen(QPen(Qt::NoPen));
        m_scene.addItem(wedgeItem);
    }
}

void SummaryPage::InitStyle(){
    this->setStyleSheet(
                        "QLabel{"
                        // "background:rgba(85,170,255,0);"
                        "color:white;"
                        "font-style:MingLiU-ExtB;"
                        "font-size:35px;"
                        "}"
                        "QCheckBox{"
                        "font-size:28px;"
                        "}"
                        "QCheckBox:checked { "
                        "color: #7FFFAA; "
                        "}"
                        "QCheckBox:unchecked { "
                        "color: #F5FFFA; "
                        "}"
                        "QCheckBox:unchecked:hover { "
                        "color: lightblue; "
                        "}"
                        "QCheckBox::indicator{"
                        "background-color:#DCDCDC;"
                        "border: 2px solid #696969;"
                        "width: 16px; height: 16px;"
                        "}"
                        "QCheckBox::indicator:unchecked:hover {"
                        " border-color: lightblue;"
                        "}"
                        "QCheckBox::indicator:checked{"
                        " border-color: lightblue;"
                        "background-color: #7FFFAA; "
                        "}"
                        "QPushButton{"
                        "background:#ced1d8;"
                        "border-style:outset;"
                        "border-radius:15px;"
                        "font-style:MingLiU-ExtB;"
                        "font-size:18px;"
                        "}"
                        "QPushButton:hover{"
                        "background-color:#F08080;"
                        "border-style:inset;"
                        "font-size:18px;"
                        "font-style:MingLiU-ExtB;"
                        "}"
                        );
}
