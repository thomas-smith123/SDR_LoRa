#ifndef LORA_TX_WORKER_H
#define LORA_TX_WORKER_H

#include <QObject>
#include <QByteArray>
#include <QVector>

#include <complex>
#include <vector>

class LoRaTxWorker : public QObject
{
    Q_OBJECT
public:
    explicit LoRaTxWorker(QObject *parent = nullptr);

    void setSampleRate(qint64 sampleRateHz);
    void setRfFrequency(double rfFrequencyHz);
    void setBandwidth(double bandwidthHz);
    void setSpreadingFactor(int sf);
    void setCodingRate(int cr);
    void setPreambleLength(int preambleLength);
    void setCrcEnabled(bool enabled);
    void setExplicitHeader(bool enabled);

public slots:
    // 预留发射入口：生成 LoRa 基带 IQ。后续接 AD9361 TX buffer 时，可在这里继续写入硬件。
    void transmitPayload(const QByteArray& payload);

signals:
    // 预留给后续硬件发送/文件保存/测试使用的复数基带 IQ。
    void txIqReady(QVector<float> iSamples, QVector<float> qSamples, qint64 sampleRateHz);
    void transmitFailed(QString reason);

private:
    std::vector<std::complex<double>> buildBasebandIq(const QByteArray& payload) const;

    qint64 sampleRateHz_ = 1000000;
    double rfFrequencyHz_ = 433e6;
    double bandwidthHz_ = 125e3;
    int sf_ = 7;
    int cr_ = 2; // LLCC68 当前配置是 CR 4/6
    int preambleLength_ = 8;
    bool crcEnabled_ = false;
    bool explicitHeader_ = true;
};

#endif // LORA_TX_WORKER_H
