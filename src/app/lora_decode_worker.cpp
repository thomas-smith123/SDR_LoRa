#include "lora_decode_worker.h"

#include "LoRaPHY.hpp"

#include <QByteArray>
#include <QDebug>

#include <cstdint>
#include <exception>
#include <vector>

LoRaDecodeWorker::LoRaDecodeWorker(QObject *parent)
    : QObject(parent)
{
}

void LoRaDecodeWorker::decodePacket(DetectedLoRaPacket packet)
{
    if (packet.symbols.isEmpty()) {
        return;
    }

    try {
        const double sampleRateHz = packet.sampleRateHz > 0 ? static_cast<double>(packet.sampleRateHz) : 1e6;
        lora::LoRaPHY phy(433e6, 7, 125e3, sampleRateHz, 8);
        phy.cr = 1;
        phy.has_header = true;
        phy.crc = true;
        phy.fast_mode = true;
        phy.ideal_fallback_enabled = false;
        phy.zero_padding_ratio = 8;
        phy.init();

        std::vector<int> symbols;
        symbols.reserve(static_cast<size_t>(packet.symbols.size()));
        for (int symbol : packet.symbols) {
            symbols.push_back(symbol);
        }

        auto decoded = phy.decode(symbols);
        QByteArray payload;
        const size_t payloadSize = decoded.checksum.size() == 2 && decoded.data.size() >= 2
                                       ? decoded.data.size() - 2
                                       : decoded.data.size();
        payload.reserve(static_cast<int>(payloadSize));
        for (size_t i = 0; i < payloadSize; ++i) {
            const uint8_t b = decoded.data[i];
            payload.append(static_cast<char>(b));
        }

        qDebug() << "CFO(Hz)" << packet.cfoHz
                 << "payload" << QString::fromLatin1(payload);
    } catch (const std::exception& e) {
        qWarning() << "LoRa decode failed:" << e.what();
    }
}