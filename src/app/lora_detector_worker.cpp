#include "lora_detector_worker.h"

#include "LoRaPHY.hpp"

#include <QDebug>

#include <algorithm>
#include <cmath>

LoRaDetectorWorker::LoRaDetectorWorker(QObject *parent)
    : QObject(parent)
{
}

void LoRaDetectorWorker::processIqFrame(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz)
{
    const int length = std::min(iSamples.size(), qSamples.size());
    if (length <= 0 || sampleRateHz <= 0) {
        return;
    }

    resetPhyIfNeeded(sampleRateHz);
    appendToRing(iSamples, qSamples);

    if (iqRing_.size() < processWindowSamples()) {
        state_ = DetectorState::Filling;
        return;
    }

    state_ = DetectorState::Searching;
    const auto streamBuffer = ringSnapshot();

    try {
        lora::LoRaPHY phy(kRfFreqHz, kSf, kBandwidthHz, static_cast<double>(sampleRateHz_), kPreambleLen);
        phy.cr = 1;
        phy.has_header = true;
        phy.crc = false;
        phy.fast_mode = true;
        phy.is_debug = false;
        phy.ideal_fallback_enabled = false;
        phy.zero_padding_ratio = kZeroPaddingRatio;
        phy.init();

        auto packets = phy.demodulate(streamBuffer);

        size_t maxConsumed = 0;
        const size_t safeCompleteLimit = streamBuffer.size() > overlapSamples()
                                             ? streamBuffer.size() - overlapSamples()
                                             : 0;
        for (const auto& packet : packets) {
            if (packet.frame_end_sample > safeCompleteLimit) {
                continue;
            }

            lora::LoRaPHY verifyPhy(kRfFreqHz, kSf, kBandwidthHz, static_cast<double>(sampleRateHz_), kPreambleLen);
            verifyPhy.cr = 1;
            verifyPhy.has_header = true;
            verifyPhy.crc = false;
            verifyPhy.fast_mode = true;
            verifyPhy.ideal_fallback_enabled = false;
            verifyPhy.zero_padding_ratio = kZeroPaddingRatio;
            verifyPhy.init();
            try {
                const auto verify = verifyPhy.decode(packet.symbols);
                if (verify.checksum.size() != 2) {
                    // 发射端关闭 PHY CRC 时这是正常情况，继续交给重复包过滤。
                } else if (!verify.crc_ok) {
                    qWarning() << "Drop LoRa packet with CRC FAIL"
                               << "symbols" << packet.symbols.size()
                               << "CFO(Hz)" << packet.cfo_hz
                               << "NETID" << packet.net_id.first << packet.net_id.second;
                    continue;
                }
            } catch (const std::exception& e) {
                qWarning() << "Drop invalid LoRa packet:" << e.what()
                           << "symbols" << packet.symbols.size()
                           << "CFO(Hz)" << packet.cfo_hz
                           << "NETID" << packet.net_id.first << packet.net_id.second;
                continue;
            }

            const quint64 absoluteEnd = absoluteBufferStart_ + packet.frame_end_sample;
            if (absoluteEnd <= lastEmittedFrameEnd_) {
                maxConsumed = std::max(maxConsumed, packet.frame_end_sample);
                continue;
            }

            DetectedLoRaPacket out;
            out.symbols.reserve(static_cast<int>(packet.symbols.size()));
            for (int symbol : packet.symbols) {
                out.symbols.append(symbol);
            }
            out.cfoHz = packet.cfo_hz;
            out.netId1 = packet.net_id.first;
            out.netId2 = packet.net_id.second;
            out.frameStartSample = absoluteBufferStart_ + packet.frame_start_sample;
            out.dataStartSample = absoluteBufferStart_ + packet.data_start_sample;
            out.frameEndSample = absoluteBufferStart_ + packet.frame_end_sample;
            out.sampleRateHz = sampleRateHz_;

            emit packetDetected(out);
            lastEmittedFrameEnd_ = out.frameEndSample;
            maxConsumed = std::max(maxConsumed, packet.frame_end_sample);
        }

        if (maxConsumed > 0) {
            trimStreamBuffer(maxConsumed);
        } else if (iqRing_.size() >= maxRingSamples()) {
            // 保守模式：未确认输出有效包前不主动裁剪 IQ，避免把跨窗口帧裁掉。
            // 如果这里持续出现，说明检测/解码速度跟不上采样输入，需要继续优化算法或降低采样率。
            qWarning() << "LoRa detector ring is full but no confirmed packet was consumed; keep IQ to avoid data loss"
                       << "ring" << iqRing_.size()
                       << "limit" << maxRingSamples();
        }
    } catch (const std::exception& e) {
        qWarning() << "LoRa detect/demod failed:" << e.what();
        if (iqRing_.size() >= maxRingSamples()) {
            qWarning() << "LoRa detector keeps IQ after exception to avoid data loss"
                       << "ring" << iqRing_.size()
                       << "limit" << maxRingSamples();
        }
    }
}

void LoRaDetectorWorker::resetPhyIfNeeded(qint64 sampleRateHz)
{
    if (sampleRateHz_ == sampleRateHz) {
        return;
    }

    sampleRateHz_ = sampleRateHz;
    iqRing_.clear();
    absoluteBufferStart_ = 0;
    lastEmittedFrameEnd_ = 0;
    state_ = DetectorState::Filling;
}

void LoRaDetectorWorker::appendToRing(const QVector<qint16>& iSamples, const QVector<qint16>& qSamples)
{
    constexpr double kAdcScale = 32768.0;
    const int length = std::min(iSamples.size(), qSamples.size());
    for (int n = 0; n < length; ++n) {
        iqRing_.emplace_back(static_cast<double>(iSamples[n]) / kAdcScale,
                             static_cast<double>(qSamples[n]) / kAdcScale);
    }

    if (iqRing_.size() > maxRingSamples()) {
        qWarning() << "LoRa detector ring exceeds target capacity; not dropping IQ"
                   << "ring" << iqRing_.size()
                   << "target" << maxRingSamples();
    }
}

std::vector<std::complex<double>> LoRaDetectorWorker::ringSnapshot() const
{
    return std::vector<std::complex<double>>(iqRing_.begin(), iqRing_.end());
}

void LoRaDetectorWorker::trimStreamBuffer(size_t consumedSamples)
{
    if (consumedSamples == 0) {
        return;
    }

    const size_t keepOverlap = overlapSamples();
    const size_t eraseCount = consumedSamples > keepOverlap ? consumedSamples - keepOverlap : 0;
    if (eraseCount == 0) {
        return;
    }

    const size_t boundedErase = std::min(eraseCount, iqRing_.size());
    for (size_t n = 0; n < boundedErase; ++n) {
        iqRing_.pop_front();
    }
    absoluteBufferStart_ += boundedErase;
}

void LoRaDetectorWorker::discardFrontSamples(size_t samples)
{
    if (samples == 0) {
        return;
    }

    const size_t boundedErase = std::min(samples, iqRing_.size());
    for (size_t n = 0; n < boundedErase; ++n) {
        iqRing_.pop_front();
    }
    absoluteBufferStart_ += boundedErase;
}

size_t LoRaDetectorWorker::processWindowSamples() const
{
    return kMinBufferedFrames * estimatedFrameSamples(kMaxPayloadBytes);
}

size_t LoRaDetectorWorker::maxRingSamples() const
{
    return kMaxBufferedFrames * estimatedFrameSamples(kMaxPayloadBytes);
}

size_t LoRaDetectorWorker::estimatedFrameSamples(int payloadBytes) const
{
    if (sampleRateHz_ <= 0) {
        return 0;
    }

    const double symbolSamples = static_cast<double>(sampleRateHz_) / kBandwidthHz * static_cast<double>(1 << kSf);
    const double sf = static_cast<double>(kSf);
    const int lowDataRateOptimize = (kSf >= 11) ? 1 : 0;
    const double numerator = 8.0 * payloadBytes - 4.0 * sf + 28.0 + 16.0 - 20.0;
    const double denominator = 4.0 * (sf - 2.0 * lowDataRateOptimize);
    const double payloadSymbols = 8.0 + std::max(std::ceil(numerator / denominator) * 5.0, 0.0);
    return static_cast<size_t>(std::ceil((kPreambleLen + 4.25 + payloadSymbols) * symbolSamples));
}

size_t LoRaDetectorWorker::overlapSamples() const
{
    if (sampleRateHz_ <= 0) {
        return 0;
    }

    const double symbolSamples = static_cast<double>(sampleRateHz_) / kBandwidthHz * static_cast<double>(1 << kSf);
    return static_cast<size_t>(std::ceil((kPreambleLen + 6.0) * symbolSamples));
}

size_t LoRaDetectorWorker::symbolSamples() const
{
    if (sampleRateHz_ <= 0) {
        return 0;
    }

    const double samples = static_cast<double>(sampleRateHz_) / kBandwidthHz * static_cast<double>(1 << kSf);
    return static_cast<size_t>(std::ceil(samples));
}