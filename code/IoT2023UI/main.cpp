#include "iot_frameless.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Iot_frameless w;
    w.ShowFrameless();
    return a.exec();
}
