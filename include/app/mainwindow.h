#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class board_read;
class LoRaDecodeWorker;
class LoRaDetectorWorker;
class QThread;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void startAd9361Receiver();
    void stopAd9361Receiver();

private:
    Ui::MainWindow *ui;
    QThread *rxThread_ = nullptr;
    board_read *rxReader_ = nullptr;
    QThread *detectorThread_ = nullptr;
    LoRaDetectorWorker *detectorWorker_ = nullptr;
    QThread *decodeThread_ = nullptr;
    LoRaDecodeWorker *decodeWorker_ = nullptr;
};
#endif // MAINWINDOW_H
