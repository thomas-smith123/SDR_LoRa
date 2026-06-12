#include "lora_detector_worker.h"

#include "LoRaPHY.hpp"

#include <QDebug>
#include <QElapsedTimer>

#include <algorithm>
#include <cmath>

LoRaDetectorWorker::LoRaDetectorWorker(QObject *parent)
    : QObject(parent)
{
    statsTimer_.start();
}

void LoRaDetectorWorker::configure(double rfFrequencyHz,
                                   int spreadingFactor,
                                   double bandwidthHz,
                                   int preambleLength,
                                   int codingRate,
                                   bool explicitHeader,
                                   bool crcEnabled,
                                   int zeroPaddingRatio,
                                   int maxPayloadBytes,
                                   int maxBufferedFrames,
                                   bool captureFrameIq)
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
    if (maxPayloadBytes > 0) {
        maxPayloadBytes_ = maxPayloadBytes;
    }
    if (maxBufferedFrames > 0) {
        maxBufferedFrames_ = maxBufferedFrames;
    }
    captureFrameIq_ = captureFrameIq;

    iqRing_.clear();
    absoluteBufferStart_ = 0;
    lastEmittedFrameEnd_ = 0;
    state_ = DetectorState::Filling;
}

void LoRaDetectorWorker::processIqFrame(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz)
{
    const int length = std::min(iSamples.size(), qSamples.size());
    if (length <= 0 || sampleRateHz <= 0) {
        return;
    }

    resetPhyIfNeeded(sampleRateHz);
    appendToRing(iSamples, qSamples);
    ++processedFrames_;

    if (iqRing_.size() < headerWindowSamples()) {
        state_ = DetectorState::Filling;
        return;
    }

    state_ = DetectorState::SearchingPreamble;
    const auto streamBuffer = ringSnapshot();
    QElapsedTimer detectTimer;
    detectTimer.start();

    try {
        lora::LoRaPHY phy(rfFrequencyHz_, spreadingFactor_, bandwidthHz_, static_cast<double>(sampleRateHz_), preambleLength_);
        phy.cr = codingRate_;
        phy.has_header = explicitHeader_;
        phy.crc = crcEnabled_;
        phy.fast_mode = true;
        phy.is_debug = false;
        phy.ideal_fallback_enabled = false;
        phy.zero_padding_ratio = zeroPaddingRatio_;
        phy.init();

        size_t searchOffset = 0;
        size_t maxConsumed = 0;
        size_t waitingForSamples = 0;
        int packetCount = 0;
        int badCandidateCount = 0;

        while (searchOffset < streamBuffer.size()) {
            auto scan = phy.scanNextFrame(streamBuffer, searchOffset);
            if (scan.status == lora::StreamingDemodResult::Status::NoPreamble) {
                maxConsumed = std::max(maxConsumed, scan.consumed_samples);
                break;
            }

            if (scan.status == lora::StreamingDemodResult::Status::NeedMoreSamples) {
                state_ = DetectorState::WaitingFullFrame;
                waitingForSamples = scan.required_samples;
                maxConsumed = std::max(maxConsumed, scan.consumed_samples);
                break;
            }

            if (scan.status == lora::StreamingDemodResult::Status::BadCandidate) {
                ++badCandidateCount;
                const size_t nextOffset = std::max(scan.consumed_samples, searchOffset + symbolSamples());
                maxConsumed = std::max(maxConsumed, scan.consumed_samples);
                searchOffset = nextOffset;
                continue;
            }

            state_ = DetectorState::DemodulatingFrame;
            const auto& packet = scan.packet;
            const quint64 absoluteEnd = absoluteBufferStart_ + packet.frame_end_sample;
            if (absoluteEnd > lastEmittedFrameEnd_) {
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
                out.frameEndSample = absoluteEnd;
                out.sampleRateHz = sampleRateHz_;
                if (captureFrameIq_) {
                    const size_t frameBegin = std::min(packet.frame_start_sample, streamBuffer.size());
                    const size_t frameEnd = std::min(packet.frame_end_sample, streamBuffer.size());
                    out.frameI.reserve(static_cast<int>(frameEnd - frameBegin));
                    out.frameQ.reserve(static_cast<int>(frameEnd - frameBegin));
                    for (size_t n = frameBegin; n < frameEnd; ++n) {
                        out.frameI.append(static_cast<float>(streamBuffer[n].real()));
                        out.frameQ.append(static_cast<float>(streamBuffer[n].imag()));
                    }
                }

                emit packetDetected(out);
                ++emittedPackets_;
                lastEmittedFrameEnd_ = out.frameEndSample;
                ++packetCount;
            }
            maxConsumed = std::max(maxConsumed, packet.frame_end_sample);
            searchOffset = std::max(packet.frame_end_sample, searchOffset + symbolSamples());
            state_ = DetectorState::SearchingPreamble;
        }

        const qint64 detectMs = detectTimer.elapsed();
        const double frameMs = 1000.0 * static_cast<double>(length) / static_cast<double>(sampleRateHz_);
        if (detectMs > 150 || detectMs > frameMs) {
            qWarning() << "LoRa detector slow"
                       << detectMs << "ms"
                       << "frameMs" << frameMs
                       << "samples" << streamBuffer.size()
                       << "ring" << iqRing_.size()
                       << "packets" << packetCount
                       << "badCandidates" << badCandidateCount
                       << "waitingFor" << waitingForSamples;
        }

        if (statsTimer_.elapsed() >= 2000) {
            qInfo() << "LoRa detector stats"
                    << "rxFrames" << processedFrames_
                    << "emittedCandidates" << emittedPackets_
                    << "ringSamples" << iqRing_.size()
                    << "state" << static_cast<int>(state_)
                    << "headerWindowSamples" << headerWindowSamples();
            processedFrames_ = 0;
            emittedPackets_ = 0;
            statsTimer_.restart();
        }

        if (maxConsumed > 0) {
            trimStreamBuffer(maxConsumed);
        } else if (state_ != DetectorState::WaitingFullFrame && iqRing_.size() > headerWindowSamples() + overlapSamples()) {
            // 没有候选且不是等待已解析 header 的完整帧时，只保留前导/同步重叠区，避免重复扫描旧噪声。
            const size_t dropCount = iqRing_.size() - headerWindowSamples();
            discardFrontSamples(dropCount);
        } else if (iqRing_.size() >= maxRingSamples()) {
            qWarning() << "LoRa detector ring is full but no confirmed packet was consumed; keep IQ for accuracy"
                       << "ring" << iqRing_.size()
                       << "limit" << maxRingSamples();
        }
    } catch (const std::exception& e) {
        qWarning() << "LoRa detect/demod failed:" << e.what();
        if (iqRing_.size() >= maxRingSamples()) {
            qWarning() << "LoRa detector keeps IQ after exception to preserve accuracy"
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
        const size_t dropCount = iqRing_.size() - maxRingSamples();
        qWarning() << "LoRa detector ring exceeds target capacity; drop oldest IQ"
                   << "ring" << iqRing_.size()
                   << "target" << maxRingSamples()
                   << "drop" << dropCount;
        discardFrontSamples(dropCount);
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

size_t LoRaDetectorWorker::headerWindowSamples() const
{
    if (sampleRateHz_ <= 0) {
        return 0;
    }

    const double samples = (preambleLength_ + 4.25 + 8.0) * static_cast<double>(symbolSamples());
    return static_cast<size_t>(std::ceil(samples));
}

size_t LoRaDetectorWorker::maxRingSamples() const
{
    return static_cast<size_t>(maxBufferedFrames_) * estimatedFrameSamples(maxPayloadBytes_);
}

size_t LoRaDetectorWorker::estimatedFrameSamples(int payloadBytes) const
{
    if (sampleRateHz_ <= 0) {
        return 0;
    }

    const double symbolSamples = static_cast<double>(sampleRateHz_) / bandwidthHz_ * static_cast<double>(1 << spreadingFactor_);
    const double sf = static_cast<double>(spreadingFactor_);
    const int lowDataRateOptimize = ((static_cast<double>(1 << spreadingFactor_) / bandwidthHz_) > 16e-3) ? 1 : 0;
    const double numerator = 8.0 * payloadBytes - 4.0 * sf + 28.0 + 16.0 - 20.0;
    const double denominator = 4.0 * (sf - 2.0 * lowDataRateOptimize);
    const double payloadSymbols = 8.0 + std::max(std::ceil(numerator / denominator) * static_cast<double>(codingRate_ + 4), 0.0);
    return static_cast<size_t>(std::ceil((preambleLength_ + 4.25 + payloadSymbols) * symbolSamples));
}

size_t LoRaDetectorWorker::overlapSamples() const
{
    if (sampleRateHz_ <= 0) {
        return 0;
    }

    const double symbolSamples = static_cast<double>(sampleRateHz_) / bandwidthHz_ * static_cast<double>(1 << spreadingFactor_);
    return static_cast<size_t>(std::ceil((preambleLength_ + 6.0) * symbolSamples));
}

size_t LoRaDetectorWorker::symbolSamples() const
{
    if (sampleRateHz_ <= 0) {
        return 0;
    }

    const double samples = static_cast<double>(sampleRateHz_) / bandwidthHz_ * static_cast<double>(1 << spreadingFactor_);
    return static_cast<size_t>(std::ceil(samples));
}