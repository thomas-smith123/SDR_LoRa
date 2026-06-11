#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "board_read.h"
#include "lora_decode_worker.h"
#include "lora_detector_worker.h"

#include <QDebug>
#include <QMetaType>
#include <QThread>
#include <QVector>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    startAd9361Receiver();
}

MainWindow::~MainWindow()
{
    stopAd9361Receiver();
    delete ui;
}

void MainWindow::startAd9361Receiver()
{
    if (rxReader_ || rxThread_) {
        return;
    }

    qRegisterMetaType<QVector<qint16>>("QVector<qint16>");
    qRegisterMetaType<qint64>("qint64");
    qRegisterMetaType<DetectedLoRaPacket>("DetectedLoRaPacket");

    rxThread_ = new QThread(this);
    detectorThread_ = new QThread(this);
    decodeThread_ = new QThread(this);
    rxReader_ = new board_read(nullptr, 1024 * 1024);
    detectorWorker_ = new LoRaDetectorWorker;
    decodeWorker_ = new LoRaDecodeWorker;
    rxReader_->ip = const_cast<char*>("ip:192.168.2.1");
    const float rxBandwidthMHz = 1.0f;
    float selectedSampleRateMHz = 0.0f;
    const QVector<float> sampleRateCandidatesMHz = {1.0f, 2.5f, 4.0f, 5.0f, 10.0f};
    for (float sampleRateMHz : sampleRateCandidatesMHz) {
        qDebug() << "Try AD9361 RX config. RF=433MHz BW=" << rxBandwidthMHz << "MHz FS=" << sampleRateMHz << "MSps";
        rxReader_->config(rxBandwidthMHz, sampleRateMHz, 433.0f);
        if (rxReader_->config_flag) {
            selectedSampleRateMHz = sampleRateMHz;
            break;
        }
    }
    rxReader_->start_flag = rxReader_->config_flag;
    rxReader_->stop_ = !rxReader_->config_flag;

    if (!rxReader_->config_flag) {
        qWarning() << "AD9361 receiver config failed; RX pipeline will not start.";
        delete rxReader_;
        delete detectorWorker_;
        delete decodeWorker_;
        delete rxThread_;
        delete detectorThread_;
        delete decodeThread_;
        rxReader_ = nullptr;
        detectorWorker_ = nullptr;
        decodeWorker_ = nullptr;
        rxThread_ = nullptr;
        detectorThread_ = nullptr;
        decodeThread_ = nullptr;
        return;
    }

    rxReader_->moveToThread(rxThread_);
    detectorWorker_->moveToThread(detectorThread_);
    decodeWorker_->moveToThread(decodeThread_);
    connect(rxThread_, &QThread::started, rxReader_, &board_read::start_read);
    connect(rxReader_, &board_read::iqFrameReady, detectorWorker_, &LoRaDetectorWorker::processIqFrame, Qt::QueuedConnection);
    connect(detectorWorker_, &LoRaDetectorWorker::packetDetected, decodeWorker_, &LoRaDecodeWorker::decodePacket, Qt::QueuedConnection);
    connect(rxThread_, &QThread::finished, rxReader_, &QObject::deleteLater);
    connect(rxThread_, &QThread::finished, rxThread_, &QObject::deleteLater);
    connect(detectorThread_, &QThread::finished, detectorWorker_, &QObject::deleteLater);
    connect(detectorThread_, &QThread::finished, detectorThread_, &QObject::deleteLater);
    connect(decodeThread_, &QThread::finished, decodeWorker_, &QObject::deleteLater);
    connect(decodeThread_, &QThread::finished, decodeThread_, &QObject::deleteLater);

    qDebug() << "Start AD9361 LoRa receiver. RF=433MHz BW=" << rxBandwidthMHz << "MHz FS=" << selectedSampleRateMHz << "MSps LoRaBW=125k SF=7 pipeline=rx->detect->decode";
    detectorThread_->start();
    decodeThread_->start();
    rxThread_->start();
}

void MainWindow::stopAd9361Receiver()
{
    if (rxReader_) {
        rxReader_->stop_ = true;
    }
    if (rxThread_) {
        rxThread_->quit();
        rxThread_->wait(3000);
    }
    if (detectorThread_) {
        detectorThread_->quit();
        detectorThread_->wait(3000);
    }
    if (decodeThread_) {
        decodeThread_->quit();
        decodeThread_->wait(3000);
    }
    rxReader_ = nullptr;
    rxThread_ = nullptr;
    detectorWorker_ = nullptr;
    detectorThread_ = nullptr;
    decodeWorker_ = nullptr;
    decodeThread_ = nullptr;
}
