#ifndef LORA_DETECTOR_WORKER_H
#define LORA_DETECTOR_WORKER_H

#include <QObject>
#include <QVector>

#include <complex>
#include <deque>
#include <vector>

struct DetectedLoRaPacket
{
    QVector<int> symbols;
    double cfoHz = 0.0;
    int netId1 = 0;
    int netId2 = 0;
    quint64 frameStartSample = 0;
    quint64 dataStartSample = 0;
    quint64 frameEndSample = 0;
    qint64 sampleRateHz = 0;
};

Q_DECLARE_METATYPE(DetectedLoRaPacket)

class LoRaDetectorWorker : public QObject
{
    Q_OBJECT

public:
    explicit LoRaDetectorWorker(QObject *parent = nullptr);

public slots:
    void processIqFrame(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz);

signals:
    void packetDetected(DetectedLoRaPacket packet);

private:
    enum class DetectorState {
        Filling,
        Searching
    };

    static constexpr double kRfFreqHz = 433e6;
    static constexpr int kSf = 7;
    static constexpr double kBandwidthHz = 125e3;
    static constexpr int kPreambleLen = 8;
    static constexpr int kZeroPaddingRatio = 8;
    static constexpr int kDetectionPayloadBytes = 64;
    static constexpr int kMaxPayloadBytes = 255;
    static constexpr size_t kMinBufferedFrames = 2;
    static constexpr size_t kMaxBufferedFrames = 16;

    void resetPhyIfNeeded(qint64 sampleRateHz);
    void appendToRing(const QVector<qint16>& iSamples, const QVector<qint16>& qSamples);
    std::vector<std::complex<double>> ringSnapshot() const;
    void trimStreamBuffer(size_t consumedSamples);
    void discardFrontSamples(size_t samples);
    size_t processWindowSamples() const;
    size_t maxRingSamples() const;
    size_t estimatedFrameSamples(int payloadBytes) const;
    size_t overlapSamples() const;
    size_t symbolSamples() const;

    DetectorState state_ = DetectorState::Filling;
    qint64 sampleRateHz_ = 0;
    std::deque<std::complex<double>> iqRing_;
    quint64 absoluteBufferStart_ = 0;
    quint64 lastEmittedFrameEnd_ = 0;
};

#endif // LORA_DETECTOR_WORKER_H