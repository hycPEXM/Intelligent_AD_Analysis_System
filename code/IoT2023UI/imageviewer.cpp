#include "imageviewer.h"

ImageViewer::ImageViewer(QWidget *parent) : QWidget(parent) {
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    previousButton = new QPushButton( this);
    previousButton->setFixedSize(50,50);
    // previousButton->setMinimumSize(50,50);
    previousButton->setEnabled(false);
    layout->addWidget(previousButton);

    imageLabel = new QLabel(this);
    imageLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(imageLabel);

    nextButton = new QPushButton(this);
    nextButton->setFixedSize(50,50);
    // nextButton->setMinimumSize(50,50);
    layout->addWidget(nextButton);

    connect(previousButton, &QPushButton::clicked, this, &ImageViewer::showPreviousImage);
    connect(nextButton, &QPushButton::clicked, this, &ImageViewer::showNextImage);

    previousButton->setStyleSheet("QPushButton{"
                        "border-image: url(:/Image/Img/xiangzuo.png);"
                        "}"
                        "QPushButton:disabled {"
                        "    border-image: url(:/Image/Img/xiangzuo_diable.png);"
                        "}"
                        "QPushButton::hover{"
                        "}"
                        "QPushButton::!hover{"
                        "}");
    nextButton->setStyleSheet("QPushButton{"
                        "    border-image: url(:/Image/Img/xiangyou.png);"
                        "}"
                        "QPushButton:disabled {"
                        "    border-image: url(:/Image/Img/xiangyou_diable.png);"
                        "}"
                        "QPushButton::hover{"
                        "}"
                        "QPushButton::!hover{"
                        "}");
}

void ImageViewer::setImages(const QStringList &imagePaths) {
    images = imagePaths;
    currentIndex = 0;
    imageLabel->resize(this->width() - 120,this->height());
    showImage(currentIndex);
}

void ImageViewer::showPreviousImage() {
    currentIndex--;
    showImage(currentIndex);

    updateButtons();
}

void ImageViewer::showNextImage() {
    currentIndex++;
    showImage(currentIndex);

    updateButtons();
}

void ImageViewer::showImage(int index) {
    if (index >= 0 && index < images.size()) {
        QImage image(images[index]);
        imageLabel->setPixmap(QPixmap::fromImage(image).scaled(imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void ImageViewer::updateButtons() {
    previousButton->setEnabled(currentIndex > 0);
    nextButton->setEnabled(currentIndex < images.size() - 1);
}
