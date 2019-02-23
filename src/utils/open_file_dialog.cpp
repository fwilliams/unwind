#include "open_file_dialog.h"

#include <QFileDialog>
#include <QApplication>
#include <QString>

std::string open_file_dialog() {
    int argc = 0;
    char* argv = nullptr;
    QApplication app(argc, &argv);
    std::string result = QFileDialog::getOpenFileName(nullptr, QString("Open Image"), QString(""), QString("Image Files (*.png *.jpg *.bmp)")).toUtf8().constData();
    QEventLoop loop;
    while (loop.processEvents());
    return result;
}
