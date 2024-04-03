#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H


#include <QtWidgets>

class ImageViewer : public QWidget {
public:
    ImageViewer(QWidget *parent = nullptr);

    void setImages(const QStringList &imagePaths);

    int getCurrentIndex(){
        return currentIndex;
    }

private slots:
    void showPreviousImage();

    void showNextImage();

private:
    void showImage(int index);

    void updateButtons();

private:
    QLabel *imageLabel;
    QPushButton *previousButton;
    QPushButton *nextButton;
    QStringList images;
    int currentIndex;
};

#endif // IMAGEVIEWER_H
