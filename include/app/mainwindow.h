#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QByteArray>
#include <QMainWindow>
#include <QString>
#include <QVector>

class board_read;
class LoRaDecodeWorker;
class LoRaDetectorWorker;
class LoRaTxWorker;
class QTimer;
class QThread;
class SpectrumCalcWorker;
class SpectrumWaterfallWidget;

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
    void transmitLoRaPayload(const QByteArray& payload);

private:
    struct AppConfig {
        QString ad9361Uri = QStringLiteral("ip:192.168.2.1");
        float rxFrequencyMHz = 433.0f;
        float rxBandwidthMHz = 1.0f;
        QVector<float> rxSampleRateCandidatesMHz = {1.0f, 2.5f, 4.0f, 5.0f, 10.0f};
        int rxBufferSamples = 65536;
        int displayMaxFps = 20;
        int displaySnapshotPoints = 4096;
        int displayFftPoints = 512;
        int displayWaterfallRows = 80;
        double spectrumMinDb = -120.0;
        double spectrumMaxDb = 20.0;
        double loraFrequencyMHz = 433.0;
        double loraBandwidthKHz = 125.0;
        int loraSpreadingFactor = 7;
        int loraPreambleSymbols = 8;
        int loraCodingRate = 1;
        bool loraExplicitHeader = true;
        bool loraPhyCrc = false;
        int loraZeroPaddingRatio = 8;
        int loraMaxPayloadBytes = 255;
        int loraMaxBufferedFrames = 16;
        bool saveSignalsEnabled = false;
        QString saveSignalsRoot = QStringLiteral("detected_iq");
        QString saveIdDirectoryPrefix;
        QString saveFileNamePrefix;
    };

    AppConfig loadConfig() const;
    void setupDisplayUi();
    void appendDecodeLog(const QString& message);
    void startSimulatedSpectrum();
    void requestCenterFrequencyShift(double deltaHz);

    Ui::MainWindow *ui;
    AppConfig config_;
    SpectrumWaterfallWidget *spectrumWidget_ = nullptr;
    QThread *rxThread_ = nullptr;
    board_read *rxReader_ = nullptr;
    QThread *detectorThread_ = nullptr;
    LoRaDetectorWorker *detectorWorker_ = nullptr;
    QThread *decodeThread_ = nullptr;
    LoRaDecodeWorker *decodeWorker_ = nullptr;
    QThread *spectrumThread_ = nullptr;
    SpectrumCalcWorker *spectrumWorker_ = nullptr;
    QThread *txThread_ = nullptr;
    LoRaTxWorker *txWorker_ = nullptr;
    QTimer *simulationTimer_ = nullptr;
    qint64 currentCenterFrequencyHz_ = 0;
};
#endif // MAINWINDOW_H
