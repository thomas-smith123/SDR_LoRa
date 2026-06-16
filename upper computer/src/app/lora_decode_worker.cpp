#include "lora_decode_worker.h"

#include "LoRaPHY.hpp"

#include <QByteArray>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QRegularExpression>
#include <QString>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <vector>

LoRaDecodeWorker::LoRaDecodeWorker(QObject *parent)
    : QObject(parent)
{
}

void LoRaDecodeWorker::configure(double rfFrequencyHz,
                                 int spreadingFactor,
                                 double bandwidthHz,
                                 int preambleLength,
                                 int codingRate,
                                 bool explicitHeader,
                                 bool crcEnabled,
                                 int zeroPaddingRatio)
{
    if (rfFrequencyHz > 0.0) {
        rfFrequencyHz_ = rfFrequencyHz;
    }
    if (spreadingFactor >= 6 && spreadingFactor <= 12) {
        spreadingFactor_ = spreadingFactor;
    }
    if (bandwidthHz > 0.0) {
        bandwidthHz_ = bandwidthHz;
    }
    if (preambleLength >= 4) {
        preambleLength_ = preambleLength;
    }
    if (codingRate >= 1 && codingRate <= 4) {
        codingRate_ = codingRate;
    }
    explicitHeader_ = explicitHeader;
    crcEnabled_ = crcEnabled;
    if (zeroPaddingRatio > 0) {
        zeroPaddingRatio_ = zeroPaddingRatio;
    }
}

void LoRaDecodeWorker::configureSignalSaving(bool enabled,
                                             const QString& rootDirectory,
                                             const QString& idDirectoryPrefix,
                                             const QString& fileNamePrefix)
{
    saveSignalsEnabled_ = enabled;
    if (!rootDirectory.trimmed().isEmpty()) {
        saveRootDirectory_ = rootDirectory.trimmed();
    }
    saveIdDirectoryPrefix_ = idDirectoryPrefix.trimmed();
    saveFileNamePrefix_ = fileNamePrefix.trimmed();
}

void LoRaDecodeWorker::decodePacket(DetectedLoRaPacket packet)
{
    if (packet.symbols.isEmpty()) {
        return;
    }

    try {
        const double sampleRateHz = packet.sampleRateHz > 0 ? static_cast<double>(packet.sampleRateHz) : 1e6;
        lora::LoRaPHY phy(rfFrequencyHz_, spreadingFactor_, bandwidthHz_, sampleRateHz, preambleLength_);
        phy.cr = codingRate_;
        phy.has_header = explicitHeader_;
        phy.crc = crcEnabled_;
        phy.fast_mode = true;
        phy.ideal_fallback_enabled = false;
        phy.zero_padding_ratio = zeroPaddingRatio_;
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

        const QString payloadText = QString::fromLatin1(payload);
        QString saveMessage;
        if (saveSignalsEnabled_) {
            const ParsedPayload parsed = parsePayload(payloadText);
            if (parsed.valid) {
                saveMessage = saveFrameNpy(packet, parsed)
                                  ? QStringLiteral("  saved_npy yes")
                                  : QStringLiteral("  saved_npy failed");
            } else {
                saveMessage = QStringLiteral("  saved_npy skipped(payload format)");
            }
        }

        const QString message = QStringLiteral("CFO(Hz) %1  payload %2%3")
                        .arg(packet.cfoHz, 0, 'f', 3)
                        .arg(payloadText, saveMessage);
        qDebug().noquote() << message;
        emit decodedLog(message);
    } catch (const std::exception& e) {
        qWarning() << "LoRa decode failed:" << e.what();
    }
}

LoRaDecodeWorker::ParsedPayload LoRaDecodeWorker::parsePayload(const QString& payloadText) const
{
    static const QRegularExpression pattern(
        QStringLiteral(R"(^\s*ID:([^,]+),\s*TEMP:([^,]+),\s*Volt:([^,]+),\s*MAG:([^\.\r\n]+)\.\s*$)"),
        QRegularExpression::CaseInsensitiveOption);
    ParsedPayload parsed;
    const QRegularExpressionMatch match = pattern.match(payloadText);
    if (!match.hasMatch()) {
        return parsed;
    }

    parsed.valid = true;
    parsed.id = match.captured(1).trimmed();
    parsed.temp = match.captured(2).trimmed();
    parsed.volt = match.captured(3).trimmed();
    parsed.mag = match.captured(4).trimmed();
    return parsed;
}

bool LoRaDecodeWorker::saveFrameNpy(const DetectedLoRaPacket& packet, const ParsedPayload& parsed) const
{
    const int count = std::min(packet.frameI.size(), packet.frameQ.size());
    if (count <= 0 || !parsed.valid || parsed.id.isEmpty()) {
        return false;
    }

    QDir root(saveRootDirectory_);
    if (!root.exists() && !root.mkpath(QStringLiteral("."))) {
        qWarning() << "Create signal save root failed:" << saveRootDirectory_;
        return false;
    }

    const QString idDirName = sanitizePathToken(saveIdDirectoryPrefix_ + parsed.id);
    if (!root.exists(idDirName) && !root.mkpath(idDirName)) {
        qWarning() << "Create ID signal directory failed:" << root.filePath(idDirName);
        return false;
    }

    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
        const QString prefix = saveFileNamePrefix_.isEmpty()
                             ? QString()
                             : sanitizePathToken(saveFileNamePrefix_) + QLatin1Char('_');
        const QString fileName = QStringLiteral("%1%2_TEMP_%3_MAG_%4_Volt_%5_CFO_%6.npy")
                               .arg(prefix,
                                   timestamp,
                                      sanitizePathToken(parsed.temp),
                                      sanitizePathToken(parsed.mag),
                                      sanitizePathToken(parsed.volt),
                                      sanitizePathToken(QString::number(packet.cfoHz, 'f', 3)));
    const QString filePath = root.filePath(idDirName + QLatin1Char('/') + fileName);

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "Open npy file failed:" << filePath << file.errorString();
        return false;
    }

    QByteArray header = QStringLiteral("{'descr': '<f4', 'fortran_order': False, 'shape': (%1, 2), }").arg(count).toLatin1();
    const int preambleSize = 10;
    int padding = 16 - ((preambleSize + header.size() + 1) % 16);
    if (padding == 16) {
        padding = 0;
    }
    header.append(QByteArray(padding, ' '));
    header.append('\n');

    QByteArray magic;
    magic.append("\x93NUMPY", 6);
    magic.append(char(1));
    magic.append(char(0));
    const quint16 headerLen = static_cast<quint16>(header.size());
    magic.append(char(headerLen & 0xFF));
    magic.append(char((headerLen >> 8) & 0xFF));

    if (file.write(magic) != magic.size() || file.write(header) != header.size()) {
        qWarning() << "Write npy header failed:" << filePath;
        return false;
    }

    QByteArray data;
    data.resize(count * 2 * static_cast<int>(sizeof(float)));
    float* out = reinterpret_cast<float*>(data.data());
    for (int n = 0; n < count; ++n) {
        out[2 * n] = packet.frameI[n];
        out[2 * n + 1] = packet.frameQ[n];
    }

    if (file.write(data) != data.size()) {
        qWarning() << "Write npy data failed:" << filePath;
        return false;
    }
    return true;
}

QString LoRaDecodeWorker::sanitizePathToken(QString text)
{
    text = text.trimmed();
    text.replace(QRegularExpression(QStringLiteral(R"([^A-Za-z0-9_.-]+)")), QStringLiteral("_"));
    while (text.startsWith(QLatin1Char('.'))) {
        text.remove(0, 1);
    }
    return text.isEmpty() ? QStringLiteral("NA") : text.left(80);
}