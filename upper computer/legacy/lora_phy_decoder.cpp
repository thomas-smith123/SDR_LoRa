#include "lora_phy_decoder.h"

#include <algorithm>
#include <cmath>
#include <fftw3.h>
#include <numeric>

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

const uint8_t kWhiteningSeq[255] = {
    0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe1, 0xc2, 0x85, 0x0b, 0x17, 0x2f, 0x5e, 0xbc, 0x78, 0xf1, 0xe3,
    0xc6, 0x8d, 0x1a, 0x34, 0x68, 0xd0, 0xa0, 0x40, 0x80, 0x01, 0x02, 0x04, 0x08, 0x11, 0x23, 0x47,
    0x8e, 0x1c, 0x38, 0x71, 0xe2, 0xc4, 0x89, 0x12, 0x25, 0x4b, 0x97, 0x2e, 0x5c, 0xb8, 0x70, 0xe0,
    0xc0, 0x81, 0x03, 0x06, 0x0c, 0x19, 0x32, 0x64, 0xc9, 0x92, 0x24, 0x49, 0x93, 0x26, 0x4d, 0x9b,
    0x37, 0x6e, 0xdc, 0xb9, 0x72, 0xe4, 0xc8, 0x90, 0x20, 0x41, 0x82, 0x05, 0x0a, 0x15, 0x2b, 0x56,
    0xad, 0x5b, 0xb6, 0x6d, 0xda, 0xb5, 0x6b, 0xd6, 0xac, 0x59, 0xb2, 0x65, 0xcb, 0x96, 0x2c, 0x58,
    0xb0, 0x61, 0xc3, 0x87, 0x0f, 0x1f, 0x3e, 0x7d, 0xfb, 0xf6, 0xed, 0xdb, 0xb7, 0x6f, 0xde, 0xbd,
    0x7a, 0xf5, 0xeb, 0xd7, 0xae, 0x5d, 0xba, 0x74, 0xe8, 0xd1, 0xa2, 0x44, 0x88, 0x10, 0x21, 0x43,
    0x86, 0x0d, 0x1b, 0x36, 0x6c, 0xd8, 0xb1, 0x63, 0xc7, 0x8f, 0x1e, 0x3c, 0x79, 0xf3, 0xe7, 0xce,
    0x9c, 0x39, 0x73, 0xe6, 0xcc, 0x98, 0x31, 0x62, 0xc5, 0x8b, 0x16, 0x2d, 0x5a, 0xb4, 0x69, 0xd2,
    0xa4, 0x48, 0x91, 0x22, 0x45, 0x8a, 0x14, 0x29, 0x52, 0xa5, 0x4a, 0x95, 0x2a, 0x54, 0xa9, 0x53,
    0xa7, 0x4e, 0x9d, 0x3b, 0x77, 0xee, 0xdd, 0xbb, 0x76, 0xec, 0xd9, 0xb3, 0x67, 0xcf, 0x9e, 0x3d,
    0x7b, 0xf7, 0xef, 0xdf, 0xbf, 0x7e, 0xfd, 0xfa, 0xf4, 0xe9, 0xd3, 0xa6, 0x4c, 0x99, 0x33, 0x66,
    0xcd, 0x9a, 0x35, 0x6a, 0xd4, 0xa8, 0x51, 0xa3, 0x46, 0x8c, 0x18, 0x30, 0x60, 0xc1, 0x83, 0x07,
    0x0e, 0x1d, 0x3a, 0x75, 0xea, 0xd5, 0xaa, 0x55, 0xab, 0x57, 0xaf, 0x5f, 0xbe, 0x7c, 0xf9, 0xf2,
    0xe5, 0xca, 0x94, 0x28, 0x50, 0xa1, 0x42, 0x84, 0x09, 0x13, 0x27, 0x4f, 0x9f, 0x3f, 0x7f
};

int modInt(int x, int m)
{
    int r = x % m;
    return r < 0 ? r + m : r;
}
}

LoRaPhyDecoder::LoRaPhyDecoder()
    : LoRaPhyDecoder(Config{})
{
}

LoRaPhyDecoder::LoRaPhyDecoder(Config config)
    : cfg_(config)
{
    init();
}

void LoRaPhyDecoder::setConfig(const Config& config)
{
    cfg_ = config;
    init();
}

void LoRaPhyDecoder::init()
{
    symbolCount_ = 1 << cfg_.sf;
    sampleNum_ = static_cast<int>(std::llround(cfg_.sampleRateHz / cfg_.bandwidthHz * symbolCount_));
    binNum_ = symbolCount_ * cfg_.zeroPaddingRatio;
    fftLen_ = sampleNum_ * cfg_.zeroPaddingRatio;
    if (cfg_.sampleRateHz != 2.0 * cfg_.bandwidthHz) {
        // LoRaPHY.m 在 demodulate() 里重采样到 2*bw；当前 C++ 先要求外部或采集配置满足 2*bw。
        sampleNum_ = 2 * symbolCount_;
        fftLen_ = sampleNum_ * cfg_.zeroPaddingRatio;
    }
    cfg_.lowDataRateOptimize = (static_cast<double>(symbolCount_) / cfg_.bandwidthHz > 16e-3);
    upchirp_ = chirp(true, 0.0, cfoHz_);
    downchirp_ = chirp(false, 0.0, cfoHz_);
}

std::vector<std::complex<double>> LoRaPhyDecoder::chirp(bool isUp, double h, double cfoHz, double tdelta) const
{
    const int N = symbolCount_;
    const double T = static_cast<double>(N) / cfg_.bandwidthHz;
    const int samplesPerSymbol = static_cast<int>(std::llround(cfg_.sampleRateHz / cfg_.bandwidthHz * N));
    const double hOrig = h;
    h = std::round(h);
    cfoHz += (hOrig - h) / N * cfg_.bandwidthHz;

    const double k = isUp ? cfg_.bandwidthHz / T : -cfg_.bandwidthHz / T;
    const double f0 = isUp ? -cfg_.bandwidthHz / 2.0 + cfoHz : cfg_.bandwidthHz / 2.0 + cfoHz;
    const int split = static_cast<int>(std::llround(samplesPerSymbol * (N - h) / N));
    const int tail = samplesPerSymbol - split;

    std::vector<std::complex<double>> y;
    y.reserve(samplesPerSymbol);
    double phi = 0.0;
    for (int n = 0; n <= split; ++n) {
        const double t = n / cfg_.sampleRateHz + tdelta;
        const double phase = 2.0 * kPi * (t * (f0 + k * T * h / N + 0.5 * k * t));
        if (n < split) {
            y.emplace_back(std::cos(phase), std::sin(phase));
        }
        phi = phase;
    }
    for (int n = 0; n < tail; ++n) {
        const double t = n / cfg_.sampleRateHz + tdelta;
        const double phase = phi + 2.0 * kPi * (t * (f0 + 0.5 * k * t));
        y.emplace_back(std::cos(phase), std::sin(phase));
    }
    return y;
}

std::pair<double, int> LoRaPhyDecoder::dechirp(const std::vector<std::complex<double>>& sig, int start, bool upSymbol) const
{
    if (start < 0 || start + sampleNum_ > static_cast<int>(sig.size())) {
        return {0.0, 0};
    }

    std::vector<std::complex<double>> in(fftLen_, {0.0, 0.0});
    const auto& ref = upSymbol ? downchirp_ : upchirp_;
    for (int i = 0; i < sampleNum_; ++i) {
        in[i] = sig[start + i] * ref[i];
    }

    fftw_complex* fftIn = reinterpret_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * fftLen_));
    fftw_complex* fftOut = reinterpret_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * fftLen_));
    for (int i = 0; i < fftLen_; ++i) {
        fftIn[i][0] = in[i].real();
        fftIn[i][1] = in[i].imag();
    }

    fftw_plan plan = fftw_plan_dft_1d(fftLen_, fftIn, fftOut, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);

    double best = -1.0;
    int bestBin = 1;
    for (int i = 0; i < binNum_; ++i) {
        const double a0 = std::hypot(fftOut[i][0], fftOut[i][1]);
        const int j = fftLen_ - binNum_ + i;
        const double a1 = std::hypot(fftOut[j][0], fftOut[j][1]);
        const double v = a0 + a1;
        if (v > best) {
            best = v;
            bestBin = i + 1; // Matlab one-based bin
        }
    }

    fftw_destroy_plan(plan);
    fftw_free(fftIn);
    fftw_free(fftOut);
    return {best, bestBin};
}

int LoRaPhyDecoder::detect(const std::vector<std::complex<double>>& sig, int start) const
{
    int ii = start;
    std::vector<int> pkBins;
    while (ii < static_cast<int>(sig.size()) - sampleNum_ * cfg_.preambleLen) {
        if (static_cast<int>(pkBins.size()) == cfg_.preambleLen - 1) {
            return ii - static_cast<int>(std::llround((pkBins.back() - 1) / static_cast<double>(cfg_.zeroPaddingRatio) * 2.0));
        }
        const auto pk = dechirp(sig, ii, true);
        if (!pkBins.empty()) {
            int diff = modInt(pkBins.back() - pk.second, binNum_);
            if (diff > binNum_ / 2) {
                diff = binNum_ - diff;
            }
            if (diff <= cfg_.zeroPaddingRatio) {
                pkBins.push_back(pk.second);
            } else {
                pkBins.assign(1, pk.second);
            }
        } else {
            pkBins.push_back(pk.second);
        }
        ii += sampleNum_;
    }
    return -1;
}

int LoRaPhyDecoder::sync(const std::vector<std::complex<double>>& sig, int start)
{
    int x = start;
    bool found = false;
    while (x < static_cast<int>(sig.size()) - sampleNum_) {
        const auto upPeak = dechirp(sig, x, true);
        const auto downPeak = dechirp(sig, x, false);
        if (std::abs(downPeak.first) > std::abs(upPeak.first)) {
            found = true;
        }
        x += sampleNum_;
        if (found) {
            break;
        }
    }
    if (!found) {
        return -1;
    }

    const auto pkd0 = dechirp(sig, x, false);
    int to = 0;
    if (pkd0.second > binNum_ / 2) {
        to = static_cast<int>(std::llround((pkd0.second - 1 - binNum_) / static_cast<double>(cfg_.zeroPaddingRatio)));
    } else {
        to = static_cast<int>(std::llround((pkd0.second - 1) / static_cast<double>(cfg_.zeroPaddingRatio)));
    }
    x += to;

    const auto pku0 = dechirp(sig, x - 4 * sampleNum_, true);
    preambleBin_ = pku0.second;
    if (preambleBin_ > binNum_ / 2) {
        cfoHz_ = (preambleBin_ - binNum_ - 1) * cfg_.bandwidthHz / binNum_;
    } else {
        cfoHz_ = (preambleBin_ - 1) * cfg_.bandwidthHz / binNum_;
    }

    const auto pku = dechirp(sig, x - sampleNum_, true);
    const auto pkd = dechirp(sig, x - sampleNum_, false);
    if (std::abs(pku.first) > std::abs(pkd.first)) {
        return x + static_cast<int>(std::llround(2.25 * sampleNum_));
    }
    return x + static_cast<int>(std::llround(1.25 * sampleNum_));
}

std::vector<LoRaPhyDecoder::Packet> LoRaPhyDecoder::demodulate(const std::vector<std::complex<float>>& iq)
{
    cfoHz_ = 0.0;
    init();

    std::vector<std::complex<double>> sig;
    sig.reserve(iq.size());
    for (auto s : iq) {
        sig.emplace_back(static_cast<double>(s.real()), static_cast<double>(s.imag()));
    }

    std::vector<Packet> packets;
    int x = 0;
    while (x < static_cast<int>(sig.size())) {
        x = detect(sig, x);
        if (x < 0) {
            break;
        }
        x = sync(sig, x);
        if (x < 0) {
            break;
        }

        Packet packet;
        packet.cfoHz = cfoHz_;
        const auto pkNetId1 = dechirp(sig, static_cast<int>(std::llround(x - 4.25 * sampleNum_)), true);
        const auto pkNetId2 = dechirp(sig, static_cast<int>(std::llround(x - 3.25 * sampleNum_)), true);
        packet.netId0 = modInt(static_cast<int>(std::llround((pkNetId1.second + binNum_ - preambleBin_) / static_cast<double>(cfg_.zeroPaddingRatio))), symbolCount_);
        packet.netId1 = modInt(static_cast<int>(std::llround((pkNetId2.second + binNum_ - preambleBin_) / static_cast<double>(cfg_.zeroPaddingRatio))), symbolCount_);

        std::vector<int> symbols;
        if (x > static_cast<int>(sig.size()) - 8 * sampleNum_) {
            break;
        }
        for (int i = 0; i < 8; ++i) {
            const auto pk = dechirp(sig, x + i * sampleNum_, true);
            symbols.push_back(modInt(static_cast<int>(std::llround((pk.second + binNum_ - preambleBin_) / static_cast<double>(cfg_.zeroPaddingRatio))), symbolCount_));
        }
        if (cfg_.hasHeader && !parseHeader(symbols, &packet)) {
            packet.error = "Invalid header checksum";
            packets.push_back(packet);
            x += 7 * sampleNum_;
            continue;
        }

        const int symNum = calcSymbolNum(cfg_.payloadLen);
        if (x > static_cast<int>(sig.size()) - symNum * sampleNum_) {
            break;
        }
        for (int i = 8; i < symNum; ++i) {
            const auto pk = dechirp(sig, x + i * sampleNum_, true);
            symbols.push_back(modInt(static_cast<int>(std::llround((pk.second + binNum_ - preambleBin_) / static_cast<double>(cfg_.zeroPaddingRatio))), symbolCount_));
        }
        x += symNum * sampleNum_;

        packet.symbols = dynamicCompensation(symbols);
        Packet decoded = decodeSymbols(packet.symbols);
        decoded.symbols = packet.symbols;
        decoded.cfoHz = packet.cfoHz;
        decoded.netId0 = packet.netId0;
        decoded.netId1 = packet.netId1;
        packets.push_back(decoded);
    }
    return packets;
}

bool LoRaPhyDecoder::parseHeader(const std::vector<int>& symbols, Packet* packet)
{
    const std::vector<int> compensated = dynamicCompensation(symbols);
    const std::vector<uint16_t> symbolsG = grayCoding(compensated);
    const std::vector<uint16_t> codewords = diagDeinterleave(std::vector<uint16_t>(symbolsG.begin(), symbolsG.begin() + 8), cfg_.sf - 2);
    const std::vector<uint8_t> nibbles = hammingDecode(codewords, 8);
    if (nibbles.size() < 5) {
        return false;
    }
    cfg_.payloadLen = static_cast<int>(nibbles[0]) * 16 + nibbles[1];
    cfg_.crc = (nibbles[2] & 1) != 0;
    cfg_.cr = nibbles[2] >> 1;

    const int expected[5] = {
        ((nibbles[0] >> 3) & 1) ^ ((nibbles[0] >> 2) & 1) ^ ((nibbles[0] >> 1) & 1) ^ ((nibbles[0] >> 0) & 1),
        ((nibbles[0] >> 3) & 1) ^ ((nibbles[1] >> 3) & 1) ^ ((nibbles[1] >> 2) & 1) ^ ((nibbles[1] >> 1) & 1) ^ ((nibbles[2] >> 0) & 1),
        ((nibbles[0] >> 2) & 1) ^ ((nibbles[1] >> 3) & 1) ^ ((nibbles[1] >> 0) & 1) ^ ((nibbles[2] >> 3) & 1) ^ ((nibbles[2] >> 1) & 1),
        ((nibbles[0] >> 1) & 1) ^ ((nibbles[1] >> 2) & 1) ^ ((nibbles[1] >> 0) & 1) ^ ((nibbles[2] >> 2) & 1) ^ ((nibbles[2] >> 1) & 1) ^ ((nibbles[2] >> 0) & 1),
        ((nibbles[0] >> 0) & 1) ^ ((nibbles[1] >> 1) & 1) ^ ((nibbles[1] >> 0) & 1) ^ ((nibbles[2] >> 2) & 1) ^ ((nibbles[2] >> 1) & 1) ^ ((nibbles[2] >> 0) & 1)
    };
    const int got[5] = {
        nibbles[3] & 1,
        (nibbles[4] >> 3) & 1,
        (nibbles[4] >> 2) & 1,
        (nibbles[4] >> 1) & 1,
        nibbles[4] & 1
    };
    const bool valid = std::equal(std::begin(expected), std::end(expected), std::begin(got));
    if (packet) {
        packet->headerValid = valid;
    }
    return valid;
}

int LoRaPhyDecoder::calcSymbolNum(int payloadLen) const
{
    const int ppm = cfg_.sf - 2 * static_cast<int>(cfg_.lowDataRateOptimize);
    const double v = (2.0 * payloadLen - cfg_.sf + 7.0 + 4.0 * cfg_.crc - 5.0 * (!cfg_.hasHeader)) / ppm;
    return static_cast<int>(8 + std::max((4 + cfg_.cr) * static_cast<int>(std::ceil(v)), 0));
}

LoRaPhyDecoder::Packet LoRaPhyDecoder::decodeSymbols(const std::vector<int>& symbols)
{
    Packet packet;
    if (symbols.size() < 8) {
        packet.error = "Not enough symbols";
        return packet;
    }

    std::vector<uint16_t> symbolsG = grayCoding(symbols);
    std::vector<uint16_t> codewords = diagDeinterleave(std::vector<uint16_t>(symbolsG.begin(), symbolsG.begin() + 8), cfg_.sf - 2);
    std::vector<uint8_t> nibbles = hammingDecode(codewords, 8);
    if (cfg_.hasHeader) {
        if (!parseHeader(symbols, &packet)) {
            packet.error = "Invalid header checksum";
            return packet;
        }
        if (nibbles.size() > 5) {
            nibbles.erase(nibbles.begin(), nibbles.begin() + 5);
        }
    }

    const int rdd = cfg_.cr + 4;
    for (int i = 8; i + rdd <= static_cast<int>(symbolsG.size()); i += rdd) {
        std::vector<uint16_t> block(symbolsG.begin() + i, symbolsG.begin() + i + rdd);
        codewords = diagDeinterleave(block, cfg_.sf - 2 * static_cast<int>(cfg_.lowDataRateOptimize));
        std::vector<uint8_t> more = hammingDecode(codewords, rdd);
        nibbles.insert(nibbles.end(), more.begin(), more.end());
    }

    std::vector<uint8_t> bytes(std::min<size_t>(255, nibbles.size() / 2));
    for (size_t i = 0; i < bytes.size(); ++i) {
        bytes[i] = static_cast<uint8_t>(nibbles[2 * i] | (nibbles[2 * i + 1] << 4));
    }

    if (cfg_.payloadLen < 0 || static_cast<size_t>(cfg_.payloadLen) > bytes.size()) {
        packet.error = "Payload length exceeds decoded bytes";
        return packet;
    }

    std::vector<uint8_t> payloadBytes(bytes.begin(), bytes.begin() + cfg_.payloadLen);
    packet.payload = dewhiten(payloadBytes);
    if (cfg_.crc) {
        if (bytes.size() < static_cast<size_t>(cfg_.payloadLen + 2)) {
            packet.error = "Missing CRC bytes";
            return packet;
        }
        packet.decodedCrc = { bytes[static_cast<size_t>(cfg_.payloadLen)], bytes[static_cast<size_t>(cfg_.payloadLen + 1)] };
        packet.calculatedCrc = calcCrc(packet.payload);
        packet.crcValid = packet.decodedCrc == packet.calculatedCrc;
    } else {
        packet.crcValid = true;
    }
    return packet;
}

std::vector<int> LoRaPhyDecoder::dynamicCompensation(const std::vector<int>& symbols) const
{
    std::vector<int> out;
    out.reserve(symbols.size());
    for (size_t i = 0; i < symbols.size(); ++i) {
        const double drift = (static_cast<double>(i) + 2.0) * symbolCount_ * cfoHz_ / cfg_.rfFreqHz;
        out.push_back(modInt(static_cast<int>(std::llround(symbols[i] - drift)), symbolCount_));
    }
    if (cfg_.lowDataRateOptimize) {
        int binOffset = 0;
        int last = 1;
        for (int& v : out) {
            const int delta = modInt(v - last, 4);
            binOffset -= delta < 2 ? delta : delta - 4;
            last = v;
            v = modInt(v + binOffset, symbolCount_);
        }
    }
    return out;
}

std::vector<uint16_t> LoRaPhyDecoder::grayCoding(std::vector<int> symbols) const
{
    for (size_t i = 0; i < symbols.size(); ++i) {
        if (i < 8 || cfg_.lowDataRateOptimize) {
            symbols[i] = static_cast<int>(std::floor(symbols[i] / 4.0));
        } else {
            symbols[i] = modInt(symbols[i] - 1, symbolCount_);
        }
    }

    std::vector<uint16_t> out;
    out.reserve(symbols.size());
    for (int s : symbols) {
        const uint16_t v = static_cast<uint16_t>(s);
        out.push_back(v ^ (v >> 1));
    }
    return out;
}

std::vector<uint16_t> LoRaPhyDecoder::diagDeinterleave(const std::vector<uint16_t>& symbols, int ppm) const
{
    std::vector<uint16_t> codewords(ppm, 0);
    for (size_t row = 0; row < symbols.size(); ++row) {
        const uint16_t shifted = circShiftRight(symbols[row], ppm, static_cast<int>(row));
        for (int col = 0; col < ppm; ++col) {
            const int bitVal = (shifted >> (ppm - 1 - col)) & 1;
            codewords[ppm - 1 - col] |= static_cast<uint16_t>(bitVal << row);
        }
    }
    return codewords;
}

std::vector<uint8_t> LoRaPhyDecoder::hammingDecode(const std::vector<uint16_t>& codewords, int rdd) const
{
    std::vector<uint8_t> nibbles;
    nibbles.reserve(codewords.size());
    for (uint16_t cw : codewords) {
        if (rdd == 7 || rdd == 8) {
            const int p2 = bit(cw, 7) ^ bit(cw, 4) ^ bit(cw, 2) ^ bit(cw, 1);
            const int p3 = bit(cw, 5) ^ bit(cw, 3) ^ bit(cw, 2) ^ bit(cw, 1);
            const int p5 = bit(cw, 6) ^ bit(cw, 4) ^ bit(cw, 3) ^ bit(cw, 2);
            cw ^= parityFix(p2 * 4 + p3 * 2 + p5);
        }
        nibbles.push_back(static_cast<uint8_t>(cw & 0x0f));
    }
    return nibbles;
}

std::vector<uint8_t> LoRaPhyDecoder::dewhiten(const std::vector<uint8_t>& bytes) const
{
    std::vector<uint8_t> out(bytes.size());
    for (size_t i = 0; i < bytes.size(); ++i) {
        out[i] = bytes[i] ^ kWhiteningSeq[i % 255];
    }
    return out;
}

std::vector<uint8_t> LoRaPhyDecoder::calcCrc(const std::vector<uint8_t>& data) const
{
    if (data.empty()) {
        return {0, 0};
    }
    if (data.size() == 1) {
        return {data.back(), 0};
    }
    if (data.size() == 2) {
        return {data.back(), data[data.size() - 2]};
    }

    uint16_t crc = 0x0000;
    for (size_t i = 0; i + 2 < data.size(); ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return { static_cast<uint8_t>(((crc >> 8) & 0xff) ^ data.back()),
             static_cast<uint8_t>((crc & 0xff) ^ data[data.size() - 2]) };
}

int LoRaPhyDecoder::bit(uint16_t word, int matlabOneBasedPos)
{
    return (word >> (matlabOneBasedPos - 1)) & 1;
}

uint16_t LoRaPhyDecoder::circShiftRight(uint16_t bits, int width, int shift)
{
    if (width <= 0) {
        return 0;
    }
    shift = modInt(shift, width);
    const uint16_t mask = static_cast<uint16_t>((1u << width) - 1u);
    bits &= mask;
    return static_cast<uint16_t>(((bits >> shift) | (bits << (width - shift))) & mask);
}

uint8_t LoRaPhyDecoder::parityFix(int p)
{
    switch (p) {
    case 3:
        return 4;
    case 5:
        return 8;
    case 6:
        return 1;
    case 7:
        return 2;
    default:
        return 0;
    }
}
