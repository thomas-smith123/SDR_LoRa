#ifndef LORA_DECODE_WORKER_H
#define LORA_DECODE_WORKER_H

#include <QByteArray>
#include <QObject>

#include "lora_detector_worker.h"

class LoRaDecodeWorker : public QObject
{
    Q_OBJECT

public:
    explicit LoRaDecodeWorker(QObject *parent = nullptr);

public slots:
    void decodePacket(DetectedLoRaPacket packet);

signals:
    void decodedLog(QString message);
};

#endif // LORA_DECODE_WORKER_H