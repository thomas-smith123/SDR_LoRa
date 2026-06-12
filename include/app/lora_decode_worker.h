#ifndef LORA_DECODE_WORKER_H
#define LORA_DECODE_WORKER_H

#include <QByteArray>
#include <QObject>
#include <QString>

#include "lora_detector_worker.h"

class LoRaDecodeWorker : public QObject
{
    Q_OBJECT

public:
    explicit LoRaDecodeWorker(QObject *parent = nullptr);

    void configure(double rfFrequencyHz,
                   int spreadingFactor,
                   double bandwidthHz,
                   int preambleLength,
                   int codingRate,
                   bool explicitHeader,
                   bool crcEnabled,
                   int zeroPaddingRatio);
    void configureSignalSaving(bool enabled,
                               const QString& rootDirectory,
                               const QString& idDirectoryPrefix,
                               const QString& fileNamePrefix);

public slots:
    void decodePacket(DetectedLoRaPacket packet);

signals:
    void decodedLog(QString message);

private:
    double rfFrequencyHz_ = 433e6;
    int spreadingFactor_ = 7;
    double bandwidthHz_ = 125e3;
    int preambleLength_ = 8;
    int codingRate_ = 1;
    bool explicitHeader_ = true;
    bool crcEnabled_ = false;
    int zeroPaddingRatio_ = 8;
    bool saveSignalsEnabled_ = false;
    QString saveRootDirectory_ = QStringLiteral("detected_iq");
    QString saveIdDirectoryPrefix_;
    QString saveFileNamePrefix_;

    struct ParsedPayload {
        bool valid = false;
        QString id;
        QString temp;
        QString mag;
        QString volt;
    };

    ParsedPayload parsePayload(const QString& payloadText) const;
    bool saveFrameNpy(const DetectedLoRaPacket& packet, const ParsedPayload& parsed) const;
    static QString sanitizePathToken(QString text);
};

#endif // LORA_DECODE_WORKER_H