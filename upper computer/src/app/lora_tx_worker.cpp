#include "lora_tx_worker.h"

#include "LoRaPHY.hpp"

#include <QDebug>

#include <algorithm>
#include <cstdint>
#include <exception>

LoRaTxWorker::LoRaTxWorker(QObject *parent)
    : QObject(parent)
{
}

void LoRaTxWorker::setSampleRate(qint64 sampleRateHz)
{
    if (sampleRateHz > 0) {
        sampleRateHz_ = sampleRateHz;
    }
}

void LoRaTxWorker::setRfFrequency(double rfFrequencyHz)
{
    if (rfFrequencyHz > 0.0) {
        rfFrequencyHz_ = rfFrequencyHz;
    }
}

void LoRaTxWorker::setBandwidth(double bandwidthHz)
{
    if (bandwidthHz > 0.0) {
        bandwidthHz_ = bandwidthHz;
    }
}

void LoRaTxWorker::setSpreadingFactor(int sf)
{
    if (sf >= 6 && sf <= 12) {
        sf_ = sf;
    }
}

void LoRaTxWorker::setCodingRate(int cr)
{
    if (cr >= 1 && cr <= 4) {
        cr_ = cr;
    }
}

void LoRaTxWorker::setPreambleLength(int preambleLength)
{
    if (preambleLength > 0) {
        preambleLength_ = preambleLength;
    }
}

void LoRaTxWorker::setCrcEnabled(bool enabled)
{
    crcEnabled_ = enabled;
}

void LoRaTxWorker::setExplicitHeader(bool enabled)
{
    explicitHeader_ = enabled;
}

void LoRaTxWorker::setZeroPaddingRatio(int zeroPaddingRatio)
{
    if (zeroPaddingRatio > 0) {
        zeroPaddingRatio_ = zeroPaddingRatio;
    }
}

void LoRaTxWorker::transmitPayload(const QByteArray& payload)
{
    try {
        const auto iq = buildBasebandIq(payload);
        QVector<float> iSamples;
        QVector<float> qSamples;
        iSamples.reserve(static_cast<int>(iq.size()));
        qSamples.reserve(static_cast<int>(iq.size()));

        for (const auto& sample : iq) {
            const double re = std::clamp(sample.real(), -1.0, 1.0);
            const double im = std::clamp(sample.imag(), -1.0, 1.0);
            iSamples.append(static_cast<float>(re));
            qSamples.append(static_cast<float>(im));
        }

        emit txIqReady(iSamples, qSamples, sampleRateHz_);
        qDebug() << "LoRa TX baseband prepared"
                 << "payload" << QString::fromLatin1(payload)
                 << "samples" << iq.size()
                 << "fs" << sampleRateHz_;
    } catch (const std::exception& e) {
        emit transmitFailed(QString::fromLatin1(e.what()));
    }
}

std::vector<std::complex<double>> LoRaTxWorker::buildBasebandIq(const QByteArray& payload) const
{
    std::vector<uint8_t> bytes;
    bytes.reserve(static_cast<size_t>(payload.size()));
    for (char b : payload) {
        bytes.push_back(static_cast<uint8_t>(b));
    }

    lora::LoRaPHY phy(rfFrequencyHz_, sf_, bandwidthHz_, static_cast<double>(sampleRateHz_), preambleLength_);
    phy.cr = cr_;
    phy.has_header = explicitHeader_;
    phy.crc = crcEnabled_;
    phy.fast_mode = true;
    phy.ideal_fallback_enabled = false;
    phy.zero_padding_ratio = zeroPaddingRatio_;
    phy.init();

    const auto symbols = phy.encode(bytes);
    return phy.modulate(symbols);
}
