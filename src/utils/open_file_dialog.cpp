#include "open_file_dialog.h"

#include <QFileDialog>
#include <QApplication>
#include <QString>



std::string open_load_file_dialog() {
    int argc = 0;
    char* argv = nullptr;
    QApplication app(argc, &argv);
    std::string result = QFileDialog::getOpenFileName(
                nullptr, QString("Open Image"), QString(""),
                QString("Project File (*.fish.pro)")).toUtf8().constData();
    QEventLoop loop;
    while (loop.processEvents());
    return result;
}

std::string open_save_file_dialog() {
    int argc = 0;
    char* argv = nullptr;
    QApplication app(argc, &argv);
    std::string result = QFileDialog::getSaveFileName(
                nullptr, QString("Save Project"), QString(""),
                QString("Project File (*.fish.pro)")).toUtf8().constData();
    QEventLoop loop;
    while (loop.processEvents());
    return result;
}

std::string open_image_file_dialog() {
    int argc = 0;
    char* argv = nullptr;
    QApplication app(argc, &argv);
    std::string result = QFileDialog::getOpenFileName(
                nullptr, QString("Open Image"), QString(""),
                QString("Image Files (*.png *.jpg *.bmp)")).toUtf8().constData();
    QEventLoop loop;
    while (loop.processEvents());
    return result;
}


std::string open_datfile_dialog() {
    int argc = 0;
    char* argv = nullptr;
    QApplication app(argc, &argv);
    std::string result = QFileDialog::getOpenFileName(
                nullptr, QString("Open Datfile"), QString(""),
                QString("Image Files (*.dat)")).toUtf8().constData();
    QEventLoop loop;
    while (loop.processEvents());
    return result;
}

std::string open_folder_dialog() {
    int argc = 0;
    char* argv = nullptr;
    QApplication app(argc, &argv);

    std::string result;
    result.assign(QFileDialog::getExistingDirectory(
                nullptr, QString("Open Directory"), QString(""),
                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks).toUtf8().constData());

    QEventLoop loop;
    while (loop.processEvents());
    return result;
}

