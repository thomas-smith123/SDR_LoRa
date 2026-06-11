#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "board_read.h"
#include "lora_decode_worker.h"
#include "lora_detector_worker.h"
#include "lora_tx_worker.h"
#include "spectrum_calc_worker.h"
#include "spectrum_waterfall_widget.h"

#include <QDebug>
#include <QDateTime>
#include <QMetaType>
#include <QPlainTextEdit>
#include <QSettings>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <QVBoxLayout>

#include <algorithm>
#include <random>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    config_ = loadConfig();
    ui->setupUi(this);
    setupDisplayUi();

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

    // 这些类型会跨线程通过 Qt queued signal/slot 传递，必须先注册到元对象系统。
    qRegisterMetaType<QVector<qint16>>("QVector<qint16>");
    qRegisterMetaType<QVector<float>>("QVector<float>");
    qRegisterMetaType<qint64>("qint64");
    qRegisterMetaType<DetectedLoRaPacket>("DetectedLoRaPacket");

    // 整个接收链路拆成多个线程：
    // 1) rxThread_       : AD9361/libiio 阻塞读取 IQ；
    // 2) detectorThread_ : LoRa 前导码检测、解调得到 symbols；
    // 3) decodeThread_   : LoRa symbols 解码成 payload 文本；
    // 4) spectrumThread_ : FFTW 频谱计算，避免 UI 线程卡顿；
    // 5) txThread_       : 预留 LoRa 发射 IQ 生成。
    rxThread_ = new QThread(this);
    detectorThread_ = new QThread(this);
    decodeThread_ = new QThread(this);
    spectrumThread_ = new QThread(this);
    txThread_ = new QThread(this);
    rxReader_ = new board_read(nullptr, static_cast<std::size_t>(config_.rxBufferSamples));
    detectorWorker_ = new LoRaDetectorWorker;
    decodeWorker_ = new LoRaDecodeWorker;
    spectrumWorker_ = new SpectrumCalcWorker;
    spectrumWorker_->configure(config_.displayFftPoints);
    txWorker_ = new LoRaTxWorker;

    // AD9361 URI 需要保持 QByteArray 生命周期直到 config() 完成；board_read 只在配置阶段使用该指针。
    QByteArray uriBytes = config_.ad9361Uri.toLatin1();
    rxReader_->ip = uriBytes.data();
    float selectedSampleRateMHz = 0.0f;
    float selectedBandwidthMHz = 0.0f;
    for (float sampleRateMHz : config_.rxSampleRateCandidatesMHz) {
        // AD9361 模拟带宽必须低于 Nyquist；这里按 0.49Fs 限制，避免 1MSps 时配置失败。
        const float maxBandwidthMHz = 0.49f * sampleRateMHz;
        float effectiveBandwidthMHz = config_.rxBandwidthMHz;
        if (effectiveBandwidthMHz > maxBandwidthMHz) {
            effectiveBandwidthMHz = maxBandwidthMHz;
        }
        qDebug() << "Try AD9361 RX config. RF=" << config_.rxFrequencyMHz << "MHz BW=" << effectiveBandwidthMHz << "MHz FS=" << sampleRateMHz << "MSps";
        rxReader_->config(effectiveBandwidthMHz, sampleRateMHz, config_.rxFrequencyMHz);
        if (rxReader_->config_flag) {
            selectedSampleRateMHz = sampleRateMHz;
            selectedBandwidthMHz = effectiveBandwidthMHz;
            rxReader_->configureDisplaySnapshot(config_.displayMaxFps, config_.displaySnapshotPoints);
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
        delete spectrumWorker_;
        delete txWorker_;
        delete rxThread_;
        delete detectorThread_;
        delete decodeThread_;
        delete spectrumThread_;
        delete txThread_;
        rxReader_ = nullptr;
        detectorWorker_ = nullptr;
        decodeWorker_ = nullptr;
        spectrumWorker_ = nullptr;
        txWorker_ = nullptr;
        rxThread_ = nullptr;
        detectorThread_ = nullptr;
        decodeThread_ = nullptr;
        spectrumThread_ = nullptr;
        txThread_ = nullptr;
        appendDecodeLog(QStringLiteral("AD9361 connection/config failed; showing simulated random spectrum."));
        startSimulatedSpectrum();
        return;
    }

    rxReader_->moveToThread(rxThread_);
    detectorWorker_->moveToThread(detectorThread_);
    decodeWorker_->moveToThread(decodeThread_);
    spectrumWorker_->moveToThread(spectrumThread_);
    txWorker_->moveToThread(txThread_);

    // RX 线程启动后进入 board_read::start_read()，该函数内部会阻塞式 iio_buffer_refill()。
    connect(rxThread_, &QThread::started, rxReader_, &board_read::start_read);

    // 显示链路：board_read 从完整接收流中按 FPS 单独切出显示快照。
    // 这一路只用于时频图，不参与 LoRa 解码；这样 UI 频谱刷新不会抢占检测用的完整 IQ。
    connect(rxReader_, &board_read::iqDisplaySnapshotReady, spectrumWorker_, &SpectrumCalcWorker::calculate, Qt::QueuedConnection);

    // 备用/回放显示链路：SpectrumWaterfallWidget 也可以主动发 IQ 快照给 FFT worker。
    // 当前主要用于保留接口，硬件接收时主要使用 iqDisplaySnapshotReady。
    connect(spectrumWidget_, &SpectrumWaterfallWidget::iqSnapshotReady, spectrumWorker_, &SpectrumCalcWorker::calculate, Qt::QueuedConnection);

    // FFT worker 算出一帧功率谱后回到 UI widget 显示线谱和瀑布图。
    connect(spectrumWorker_, &SpectrumCalcWorker::spectrumReady, spectrumWidget_, &SpectrumWaterfallWidget::showSpectrum, Qt::QueuedConnection);

    // 解码链路：完整 RX IQ frame 送入 detector，不使用显示快照，保证解码数据连续。
    connect(rxReader_, &board_read::iqFrameReady, detectorWorker_, &LoRaDetectorWorker::processIqFrame, Qt::QueuedConnection);

    // detector 只输出已检测/解调出的 LoRa symbols；payload 解码放到独立线程，避免阻塞下一轮检测。
    connect(detectorWorker_, &LoRaDetectorWorker::packetDetected, decodeWorker_, &LoRaDecodeWorker::decodePacket, Qt::QueuedConnection);

    // 解码结果回到主线程追加到窗口右侧/下方 log；appendDecodeLog 内部会加时间戳。
    connect(decodeWorker_, &LoRaDecodeWorker::decodedLog, this, &MainWindow::appendDecodeLog, Qt::QueuedConnection);

    // 线程退出时释放 worker/thread 对象。worker 没有父对象，必须依赖 deleteLater 在所属线程中安全释放。
    connect(rxThread_, &QThread::finished, rxReader_, &QObject::deleteLater);
    connect(rxThread_, &QThread::finished, rxThread_, &QObject::deleteLater);
    connect(detectorThread_, &QThread::finished, detectorWorker_, &QObject::deleteLater);
    connect(detectorThread_, &QThread::finished, detectorThread_, &QObject::deleteLater);
    connect(decodeThread_, &QThread::finished, decodeWorker_, &QObject::deleteLater);
    connect(decodeThread_, &QThread::finished, decodeThread_, &QObject::deleteLater);
    connect(spectrumThread_, &QThread::finished, spectrumWorker_, &QObject::deleteLater);
    connect(spectrumThread_, &QThread::finished, spectrumThread_, &QObject::deleteLater);

    // TX 链路目前只生成 LoRa IQ 并打印状态；后续若接入 AD9361 TX，可在这里把 txIqReady 接到实际发射器。
    connect(txWorker_, &LoRaTxWorker::txIqReady, this, [](const QVector<float>& iSamples, const QVector<float>& qSamples, qint64 sampleRateHz) {
        qDebug() << "LoRa TX interface ready"
                 << "I" << iSamples.size()
                 << "Q" << qSamples.size()
                 << "fs" << sampleRateHz;
    });
    connect(txWorker_, &LoRaTxWorker::transmitFailed, this, [](const QString& reason) {
        qWarning() << "LoRa TX failed:" << reason;
    });
    connect(txThread_, &QThread::finished, txWorker_, &QObject::deleteLater);
    connect(txThread_, &QThread::finished, txThread_, &QObject::deleteLater);

    qDebug() << "Start AD9361 LoRa receiver. RF=" << config_.rxFrequencyMHz << "MHz BW=" << selectedBandwidthMHz << "MHz FS=" << selectedSampleRateMHz << "MSps LoRaBW=125k SF=7 pipeline=rx->detect->decode";
    // 先启动处理线程，再启动 RX 线程；避免 RX 一开始发出 queued signal 时接收线程尚未运行。
    detectorThread_->start();
    decodeThread_->start();
    spectrumThread_->start();
    txThread_->start();
    rxThread_->start();
}

MainWindow::AppConfig MainWindow::loadConfig() const
{
    AppConfig cfg;
    // 所有可调参数集中放在 config.ini，避免采样率、buffer、FPS 等运行参数写死在代码里。
    QSettings settings(QStringLiteral("config.ini"), QSettings::IniFormat);

    cfg.ad9361Uri = settings.value(QStringLiteral("ad9361/uri"), cfg.ad9361Uri).toString();
    cfg.rxFrequencyMHz = settings.value(QStringLiteral("ad9361/rx_frequency_mhz"), cfg.rxFrequencyMHz).toFloat();
    cfg.rxBandwidthMHz = settings.value(QStringLiteral("ad9361/rx_bandwidth_mhz"), cfg.rxBandwidthMHz).toFloat();
    cfg.rxBufferSamples = settings.value(QStringLiteral("ad9361/rx_buffer_samples"), cfg.rxBufferSamples).toInt();

    const QString rateText = settings.value(QStringLiteral("ad9361/rx_sample_rate_candidates_msps"), QStringLiteral("1.0,2.5,4.0,5.0,10.0")).toString();
    QVector<float> rates;
    // 采样率候选按配置文件顺序尝试；例如 1MSps 失败时会自动尝试后续采样率。
    for (const QString& item : rateText.split(',', Qt::SkipEmptyParts)) {
        bool ok = false;
        const float rate = item.trimmed().toFloat(&ok);
        if (ok && rate > 0.0f) {
            rates.append(rate);
        }
    }
    if (!rates.isEmpty()) {
        cfg.rxSampleRateCandidatesMHz = rates;
    }

    cfg.displayMaxFps = settings.value(QStringLiteral("display/max_fps"), cfg.displayMaxFps).toInt();
    cfg.displaySnapshotPoints = settings.value(QStringLiteral("display/snapshot_points"), cfg.displaySnapshotPoints).toInt();
    cfg.displayFftPoints = settings.value(QStringLiteral("display/fft_points"), cfg.displayFftPoints).toInt();
    cfg.displayWaterfallRows = settings.value(QStringLiteral("display/waterfall_rows"), cfg.displayWaterfallRows).toInt();
    return cfg;
}

void MainWindow::setupDisplayUi()
{
    // 频谱/瀑布图 widget 动态插入到 .ui 中预留的 spectrumLayout。
    // 显示参数来自 config.ini，保证接收与显示刷新策略可单独调整。
    spectrumWidget_ = new SpectrumWaterfallWidget(this);
    spectrumWidget_->configureDisplay(config_.displayMaxFps, config_.displaySnapshotPoints, config_.displayWaterfallRows, config_.displayFftPoints);
    ui->spectrumLayout->addWidget(spectrumWidget_);
    ui->mainSplitter->setStretchFactor(0, 4);
    ui->mainSplitter->setStretchFactor(1, 1);
}

void MainWindow::appendDecodeLog(const QString& message)
{
    // 所有解码日志统一在 UI 线程加时间戳，避免各 worker 输出格式不一致。
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"));
    ui->decodeLogEdit->appendPlainText(QStringLiteral("[%1] %2").arg(timestamp, message));
}

void MainWindow::startSimulatedSpectrum()
{
    // 无 AD9361 或连接失败时，用随机噪声频谱维持 UI 刷新，方便验证界面和瀑布图性能。
    if (simulationTimer_) {
        simulationTimer_->stop();
        simulationTimer_->deleteLater();
    }

    simulationTimer_ = new QTimer(this);
    const int intervalMs = 1000 / std::max(1, config_.displayMaxFps);
    const int points = std::max(128, config_.displayFftPoints);
    const qint64 sampleRateHz = static_cast<qint64>(config_.rxSampleRateCandidatesMHz.isEmpty()
                                                       ? 1000000.0
                                                       : config_.rxSampleRateCandidatesMHz.first() * 1000000.0);

    connect(simulationTimer_, &QTimer::timeout, this, [this, points, sampleRateHz]() {
        static std::default_random_engine randomEngine;
        static std::uniform_real_distribution<double> noiseDistribution(-95.0, -55.0);

        QVector<double> spectrumDb(points);
        for (int i = 0; i < points; ++i) {
            spectrumDb[i] = noiseDistribution(randomEngine);
        }
        spectrumWidget_->showSpectrum(std::move(spectrumDb), sampleRateHz);
    });
    simulationTimer_->start(intervalMs);
}

void MainWindow::transmitLoRaPayload(const QByteArray& payload)
{
    if (!txWorker_) {
        qWarning() << "LoRa TX interface is not initialized";
        return;
    }

    // 跨线程调用 TX worker，保持 UI 线程不直接执行 IQ 生成。
    QMetaObject::invokeMethod(txWorker_, "transmitPayload", Qt::QueuedConnection, Q_ARG(QByteArray, payload));
}

void MainWindow::stopAd9361Receiver()
{
    // 析构/关闭时按线程逐个 quit + wait，确保 libiio 读取、FFT、检测、解码都安全退出。
    if (simulationTimer_) {
        simulationTimer_->stop();
        simulationTimer_->deleteLater();
        simulationTimer_ = nullptr;
    }
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
    if (spectrumThread_) {
        spectrumThread_->quit();
        spectrumThread_->wait(3000);
    }
    if (txThread_) {
        txThread_->quit();
        txThread_->wait(3000);
    }
    rxReader_ = nullptr;
    rxThread_ = nullptr;
    detectorWorker_ = nullptr;
    detectorThread_ = nullptr;
    decodeWorker_ = nullptr;
    decodeThread_ = nullptr;
    spectrumWorker_ = nullptr;
    spectrumThread_ = nullptr;
    txWorker_ = nullptr;
    txThread_ = nullptr;
}
