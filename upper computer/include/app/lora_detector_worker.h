#ifndef LORA_DETECTOR_WORKER_H
#define LORA_DETECTOR_WORKER_H

#include <QObject>
#include <QVector>
#include <QElapsedTimer>

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
    QVector<float> frameI; // 完整 LoRa 帧 IQ，用于解码成功后按 payload 保存
    QVector<float> frameQ;
};

Q_DECLARE_METATYPE(DetectedLoRaPacket)

class LoRaDetectorWorker : public QObject
{
    Q_OBJECT

public:
    explicit LoRaDetectorWorker(QObject *parent = nullptr);

    void configure(double rfFrequencyHz,
                   int spreadingFactor,
                   double bandwidthHz,
                   int preambleLength,
                   int codingRate,
                   bool explicitHeader,
                   bool crcEnabled,
                   int zeroPaddingRatio,
                   int maxPayloadBytes,
                   int maxBufferedFrames,
                   bool captureFrameIq);

public slots:
    void processIqFrame(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz);

signals:
    void packetDetected(DetectedLoRaPacket packet);

private:
    enum class DetectorState {
        Filling,
        SearchingPreamble,
        WaitingFullFrame,
        DemodulatingFrame
    };

    void resetPhyIfNeeded(qint64 sampleRateHz);
    void appendToRing(const QVector<qint16>& iSamples, const QVector<qint16>& qSamples);
    std::vector<std::complex<double>> ringSnapshot() const;
    void trimStreamBuffer(size_t consumedSamples);
    void discardFrontSamples(size_t samples);
    size_t headerWindowSamples() const;
    size_t maxRingSamples() const;
    size_t estimatedFrameSamples(int payloadBytes) const;
    size_t overlapSamples() const;
    size_t symbolSamples() const;

    DetectorState state_ = DetectorState::Filling;
    qint64 sampleRateHz_ = 0;
    double rfFrequencyHz_ = 433e6;
    int spreadingFactor_ = 7;
    double bandwidthHz_ = 125e3;
    int preambleLength_ = 8;
    int codingRate_ = 1;
    bool explicitHeader_ = true;
    bool crcEnabled_ = false;
    int zeroPaddingRatio_ = 8;
    int maxPayloadBytes_ = 255;
    int maxBufferedFrames_ = 16;
    bool captureFrameIq_ = false;
    std::deque<std::complex<double>> iqRing_;
    quint64 absoluteBufferStart_ = 0;
    quint64 lastEmittedFrameEnd_ = 0;
    qint64 processedFrames_ = 0;
    qint64 emittedPackets_ = 0;
    QElapsedTimer statsTimer_;
};

#endif // LORA_DETECTOR_WORKER_H