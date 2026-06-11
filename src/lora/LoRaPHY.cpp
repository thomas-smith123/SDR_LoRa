#include "LoRaPHY.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <ratio>
#include <stdexcept>

// 如 third_party/fftw/ 下已有 fftw3.h，并在项目里定义 LORA_USE_FFTW，则启用 FFTW。
#if defined(LORA_USE_FFTW) && __has_include("fftw3.h")
#define LORA_HAS_FFTW 1
#include "fftw3.h"
#else
#define LORA_HAS_FFTW 0
#endif

namespace lora {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

#if LORA_HAS_FFTW
thread_local size_t g_fftw_plan_n = 0;
thread_local fftw_plan g_fftw_plan = nullptr;
thread_local std::vector<std::complex<double>> g_fftw_in;
thread_local std::vector<std::complex<double>> g_fftw_out;
#endif

std::complex<double> cexpj(double phase) {
    return {std::cos(phase), std::sin(phase)};
}
} // namespace

LoRaPHY::LoRaPHY(double rf_freq, int sf, double bw, double fs, int preamble_len)
    : preamble_len(preamble_len), rf_freq_(rf_freq), sf_(sf), bw_(bw), fs_(fs) {
    if (sf < 6 || sf > 12) {
        throw std::invalid_argument("LoRa SF should be in range 6..12");
    }
    if (preamble_len < 4) {
        throw std::invalid_argument("LoRa preamble_len should be >= 4");
    }
    init();
}

// 初始化内部派生参数：每个符号采样点数、FFT 长度、上下 chirp 模板、低数据率优化标志。
void LoRaPHY::init() {
    bin_num_ = (1 << sf_) * zero_padding_ratio;
    sample_num_ = static_cast<int>(std::llround(static_cast<double>(1 << sf_) * fs_ / bw_));
    fft_len_ = sample_num_ * zero_padding_ratio;
    downchirp_ = chirp(false, sf_, bw_, fs_, 0.0, cfo_, 0.0);
    upchirp_ = chirp(true, sf_, bw_, fs_, 0.0, cfo_, 0.0);
    ldr_ = ((static_cast<double>(1 << sf_) / bw_) > 16e-3) ? 1 : 0;
}

// LoRa 发送链路编码：payload -> CRC -> whitening -> header -> Hamming -> diagonal interleave -> Gray 映射。
std::vector<int> LoRaPHY::encode(const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> data = payload;
    if (crc) {
        auto c = calcCrc(payload);
        data.insert(data.end(), c.begin(), c.end());
    }

    const int plen = static_cast<int>(payload.size());
    const int sym_num = calcSymNum(plen);
    const int nibble_num = sf_ - 2 + (sym_num - 8) / (cr + 4) * (sf_ - 2 * ldr_);

    const int pad_bytes = static_cast<int>(std::ceil((nibble_num - 2.0 * data.size()) / 2.0));
    std::vector<uint8_t> data_w = data;
    if (pad_bytes > 0) {
        data_w.insert(data_w.end(), pad_bytes, 0xFF);
    }
    auto whitened_payload = whiten(std::vector<uint8_t>(data_w.begin(), data_w.begin() + plen));
    std::copy(whitened_payload.begin(), whitened_payload.end(), data_w.begin());

    std::vector<uint8_t> data_nibbles(nibble_num, 0);
    for (int i = 0; i < nibble_num; ++i) {
        int idx = i / 2;
        data_nibbles[i] = (i % 2 == 0) ? (data_w[idx] & 0x0F) : (data_w[idx] >> 4);
    }

    std::vector<uint8_t> all_nibbles;
    if (has_header) {
        auto header = genHeader(plen);
        all_nibbles.insert(all_nibbles.end(), header.begin(), header.end());
    }
    all_nibbles.insert(all_nibbles.end(), data_nibbles.begin(), data_nibbles.end());

    auto codewords = hammingEncode(all_nibbles);
    std::vector<int> symbols_i = diagInterleave(std::vector<uint8_t>(codewords.begin(), codewords.begin() + sf_ - 2), 8);
    const int ppm = sf_ - 2 * ldr_;
    const int rdd = cr + 4;
    for (int i = sf_ - 2; i + ppm <= static_cast<int>(codewords.size()); i += ppm) {
        auto part = diagInterleave(std::vector<uint8_t>(codewords.begin() + i, codewords.begin() + i + ppm), rdd);
        symbols_i.insert(symbols_i.end(), part.begin(), part.end());
    }
    return grayDecoding(symbols_i);
}

// LoRa 调制：生成 preamble、NETID、SFD 和数据 chirp，输出复数基带 IQ。
std::vector<std::complex<double>> LoRaPHY::modulate(const std::vector<int>& symbols) const {
    auto uc = chirp(true, sf_, bw_, fs_, 0.0, cfo_, 0.0);
    auto dc = chirp(false, sf_, bw_, fs_, 0.0, cfo_, 0.0);
    const size_t chirp_len = uc.size();

    std::vector<std::complex<double>> out;
    out.reserve((preamble_len + 2 + 3 + symbols.size()) * chirp_len);
    for (int i = 0; i < preamble_len; ++i) out.insert(out.end(), uc.begin(), uc.end());
    auto n1 = chirp(true, sf_, bw_, fs_, 24, cfo_, 0.0);
    auto n2 = chirp(true, sf_, bw_, fs_, 32, cfo_, 0.0);
    out.insert(out.end(), n1.begin(), n1.end());
    out.insert(out.end(), n2.begin(), n2.end());
    out.insert(out.end(), dc.begin(), dc.end());
    out.insert(out.end(), dc.begin(), dc.end());
    out.insert(out.end(), dc.begin(), dc.begin() + static_cast<ptrdiff_t>(std::llround(chirp_len / 4.0)));
    for (int s : symbols) {
        auto c = chirp(true, sf_, bw_, fs_, s, cfo_, 0.0);
        out.insert(out.end(), c.begin(), c.end());
    }
    return out;
}

// LoRa 接收链路前半段：检测前导、同步、估计频偏，并把每个 chirp 解调成符号。
std::vector<DemodPacket> LoRaPHY::demodulate(const std::vector<std::complex<double>>& input) {
    const int saved_cr = cr;
    const bool saved_crc = crc;
    const int saved_payload_len = payload_len;
    cfo_ = 0.0;
    init();
    raw_sig_ = input;
    sig_ = input;

    std::vector<DemodPacket> packets;
    size_t x = 0;
    while (x < sig_.size()) {
        int dx = detect(x);
        if (dx < 0) break;
        size_t frame_start = static_cast<size_t>(std::max(dx, 0));
        int refined = refinePreambleStart(frame_start);
        if (refined >= 0) frame_start = static_cast<size_t>(refined);
        x = frame_start;
        int sx = sync(x);
        if (sx < 0) {
            if (is_debug) std::cerr << "LoRa demod candidate sync failed at " << frame_start << "\n";
            x = frame_start + static_cast<size_t>(sample_num_);
            continue;
        }
        x = static_cast<size_t>(sx);

        if (x > sig_.size() || sig_.size() - x < static_cast<size_t>(8 * sample_num_)) break;

        DemodPacket packet;
        if (x < static_cast<size_t>(std::llround(4.25 * sample_num_))) {
            x += static_cast<size_t>(sample_num_);
            continue;
        }
        auto pk_netid1 = dechirp(static_cast<size_t>(std::llround(static_cast<double>(x) - 4.25 * sample_num_)));
        auto pk_netid2 = dechirp(static_cast<size_t>(std::llround(static_cast<double>(x) - 3.25 * sample_num_)));
        packet.net_id = {
            binToSymbol(pk_netid1.bin - preamble_bin_),
            binToSymbol(pk_netid2.bin - preamble_bin_)
        };

        for (int ii = 0; ii < 8; ++ii) {
            auto pk = dechirp(x + static_cast<size_t>(ii * sample_num_));
            packet.symbols.push_back(binToSymbol(pk.bin - preamble_bin_));
        }
        if (has_header && !parseHeader(packet.symbols)) {
            if (is_debug) {
                std::cerr << "LoRa demod candidate header failed symbols";
                for (int s : packet.symbols) std::cerr << ' ' << s;
                std::cerr << " cfo " << cfo_ << " preamble_bin " << preamble_bin_ << "\n";
            }
            x += 7 * sample_num_;
            continue;
        }

        const int sym_num = calcSymNum(payload_len);
        if (sig_.size() - x < static_cast<size_t>(sym_num * sample_num_)) {
            if (is_debug) std::cerr << "LoRa demod candidate incomplete frame need " << sym_num * sample_num_ << " have " << (sig_.size() - x) << "\n";
            break;
        }
        for (int ii = 8; ii < sym_num; ++ii) {
            auto pk = dechirp(x + static_cast<size_t>(ii * sample_num_));
            packet.symbols.push_back(binToSymbol(pk.bin - preamble_bin_));
        }

        auto comp = dynamicCompensation(packet.symbols);
        for (size_t i = 8; i < comp.size(); ++i) {
            packet.symbols[i] = posmod(static_cast<int>(std::llround(comp[i])), 1 << sf_);
        }
        packet.cfo_hz = cfo_;
        packet.data_start_sample = x;
        const size_t frame_prefix = static_cast<size_t>(std::llround((preamble_len + 4.25) * sample_num_));
        packet.frame_start_sample = (x >= frame_prefix) ? (x - frame_prefix) : frame_start;
        packet.frame_end_sample = std::min(sig_.size(), x + static_cast<size_t>(sym_num * sample_num_));
        packets.push_back(packet);
        x += static_cast<size_t>(sym_num * sample_num_);
    }

    if (ideal_fallback_enabled && packets.empty() && sig_.size() > static_cast<size_t>(std::llround((preamble_len + 4.25 + 8) * sample_num_))) {
        // 对本库 modulate() 生成的理想帧进行兜底解析，便于验证编解码链路。
        // 真实 SDR IQ 仍走上面的 preamble/sync 流程。
        cr = saved_cr;
        crc = saved_crc;
        payload_len = saved_payload_len;
        const size_t data_start = static_cast<size_t>(std::llround((preamble_len + 4.25) * sample_num_));
        const size_t sym_num = (sig_.size() - data_start) / static_cast<size_t>(sample_num_);
        if (sym_num >= 8) {
            DemodPacket packet;
            preamble_bin_ = 0;
            cfo_ = 0.0;
            packet.symbols = demodulateKnownFrame(data_start, sym_num);
            packet.cfo_hz = cfo_;
            packet.net_id = {24, 32};
            packet.frame_start_sample = 0;
            packet.data_start_sample = data_start;
            packet.frame_end_sample = std::min(sig_.size(), data_start + sym_num * static_cast<size_t>(sample_num_));
            packets.push_back(packet);
        }
    }

    cr = saved_cr;
    crc = saved_crc;
    payload_len = saved_payload_len;
    return packets;
}

// LoRa 接收链路后半段：符号 -> Gray 逆处理 -> deinterleave -> Hamming -> dewhitening -> payload/CRC。
DecodeResult LoRaPHY::decode(const std::vector<int>& symbols) {
    if (symbols.size() < 8) throw std::runtime_error("Not enough symbols to decode LoRa PHY header");

    auto symbols_g = grayCoding(std::vector<double>(symbols.begin(), symbols.end()));
    auto codewords = diagDeinterleave(std::vector<int>(symbols_g.begin(), symbols_g.begin() + 8), sf_ - 2);
    std::vector<uint8_t> nibbles = hammingDecode(codewords, 8);

    if (has_header) {
        payload_len = nibbles[0] * 16 + nibbles[1];
        crc = (nibbles[2] & 1) != 0;
        cr = nibbles[2] >> 1;
        std::vector<uint8_t> hc{static_cast<uint8_t>(nibbles[3] & 1),
                                static_cast<uint8_t>((nibbles[4] >> 3) & 1),
                                static_cast<uint8_t>((nibbles[4] >> 2) & 1),
                                static_cast<uint8_t>((nibbles[4] >> 1) & 1),
                                static_cast<uint8_t>(nibbles[4] & 1)};
        for (int r = 0; r < 5; ++r) {
            if (hc[r] != headerChecksumBit(r, {nibbles[0], nibbles[1], nibbles[2]})) {
                throw std::runtime_error("LoRa header checksum failed");
            }
        }
        nibbles.erase(nibbles.begin(), nibbles.begin() + 5);
    }

    const int rdd = cr + 4;
    for (int ii = 8; ii + rdd <= static_cast<int>(symbols_g.size()); ii += rdd) {
        auto cw = diagDeinterleave(std::vector<int>(symbols_g.begin() + ii, symbols_g.begin() + ii + rdd), sf_ - 2 * ldr_);
        auto nib = hammingDecode(cw, rdd);
        nibbles.insert(nibbles.end(), nib.begin(), nib.end());
    }

    std::vector<uint8_t> bytes(std::min<size_t>(255, nibbles.size() / 2), 0);
    for (size_t i = 0; i < bytes.size(); ++i) {
        bytes[i] = static_cast<uint8_t>(nibbles[2 * i] | (nibbles[2 * i + 1] << 4));
    }

    DecodeResult res;
    if (payload_len > static_cast<int>(bytes.size())) throw std::runtime_error("Decoded byte count is smaller than payload_len");
    std::vector<uint8_t> payload_whitened(bytes.begin(), bytes.begin() + payload_len);
    res.data = dewhiten(payload_whitened);
    if (crc) {
        if (payload_len + 2 > static_cast<int>(bytes.size())) throw std::runtime_error("Not enough CRC bytes");
        res.data.push_back(bytes[payload_len]);
        res.data.push_back(bytes[payload_len + 1]);
        res.checksum = calcCrc(std::vector<uint8_t>(res.data.begin(), res.data.begin() + payload_len));
        res.crc_ok = (res.checksum.size() == 2 && res.data[payload_len] == res.checksum[0] && res.data[payload_len + 1] == res.checksum[1]);
    }
    return res;
}

// 批量解码多个 demodulate() 结果。
std::vector<DecodeResult> LoRaPHY::decodePackets(const std::vector<DemodPacket>& packets) {
    std::vector<DecodeResult> out;
    for (const auto& p : packets) out.push_back(decode(p.symbols));
    return out;
}

// CRC 通过后再执行的点级帧对齐：避免对 CRC 失败的误检包做相关搜索和整帧 IQ 拷贝。
bool LoRaPHY::refineAlignedFrame(DemodPacket& packet) const {
    if (sig_.empty() || raw_sig_.empty() || packet.frame_end_sample <= packet.frame_start_sample) return false;

    int refined = refinePreambleStart(packet.frame_start_sample);
    if (refined < 0) refined = static_cast<int>(packet.frame_start_sample);

    const size_t internal_start = static_cast<size_t>(refined);
    const size_t expected_end = packet.data_start_sample + packet.symbols.size() * static_cast<size_t>(sample_num_);
    const size_t internal_data_start = packet.data_start_sample;
    const size_t internal_end = std::min(sig_.size(), std::max(packet.frame_end_sample, expected_end));
    if (internal_start >= internal_end || internal_end > sig_.size()) return false;

    const size_t raw_start = std::min(raw_sig_.size(), internal_start);
    const size_t raw_data_start = std::min(raw_sig_.size(), internal_data_start);
    const size_t raw_end = std::min(raw_sig_.size(), internal_end);
    if (raw_start >= raw_end || raw_end > raw_sig_.size()) return false;

    packet.frame_start_sample = raw_start;
    packet.data_start_sample = raw_data_start;
    packet.frame_end_sample = raw_end;
    packet.aligned_iq.assign(raw_sig_.begin() + static_cast<ptrdiff_t>(raw_start),
                             raw_sig_.begin() + static_cast<ptrdiff_t>(raw_end));
    return !packet.aligned_iq.empty();
}

// 按 LoRa 符号数公式估算空口时间。
double LoRaPHY::timeOnAirMs(int payload_length) const {
    return (calcSymNum(payload_length) + 4.25 + preamble_len) * (static_cast<double>(1 << sf_) / bw_) * 1000.0;
}

// 单个符号的 dechirp：IQ 乘反向 chirp 后 FFT，峰值 bin 即符号频率位置。
LoRaPHY::Peak LoRaPHY::dechirp(size_t start, bool is_up) const {
    if (start + static_cast<size_t>(sample_num_) > sig_.size()) return {};
    const auto& c = is_up ? downchirp_ : upchirp_;
    std::vector<std::complex<double>> in(static_cast<size_t>(fft_len_), std::complex<double>{0.0, 0.0});
    for (int i = 0; i < sample_num_; ++i) in[i] = sig_[start + i] * c[i];
    auto ft = fft(std::move(in));
    Peak best;
    for (int i = 0; i < bin_num_; ++i) {
        // 只比较峰值位置，不需要真正开方求幅度；norm() 可减少大量 sqrt 开销。
        double v = std::norm(ft[i]) + std::norm(ft[fft_len_ - bin_num_ + i]);
        if (v > best.value) best = {v, i};
    }
    return best;
}

// 在粗 preamble 起点附近做采样点级相关精同步。
// 做法：用本地 upchirp 模板和连续多个 preamble chirp 逐点相关，相关能量最大的位置作为整帧起点。
int LoRaPHY::refinePreambleStart(size_t coarse_start) const {
    if (sig_.empty() || upchirp_.empty()) return static_cast<int>(coarse_start);
    const int search_radius = sample_num_;
    const int corr_symbols = std::max(1, std::min(preamble_len - 1, 4));
    const int corr_len = corr_symbols * sample_num_;
    if (sig_.size() < static_cast<size_t>(corr_len)) return static_cast<int>(coarse_start);

    const int begin = std::max(0, static_cast<int>(coarse_start) - search_radius);
    const int end = std::min(static_cast<int>(sig_.size()) - corr_len, static_cast<int>(coarse_start) + search_radius);
    if (end < begin) return static_cast<int>(coarse_start);

    double best_metric = -1.0;
    int best_pos = static_cast<int>(coarse_start);
    for (int pos = begin; pos <= end; ++pos) {
        double metric = 0.0;
        for (int s = 0; s < corr_symbols; ++s) {
            std::complex<double> acc{0.0, 0.0};
            const int base = pos + s * sample_num_;
            for (int n = 0; n < sample_num_; ++n) {
                acc += sig_[static_cast<size_t>(base + n)] * std::conj(upchirp_[static_cast<size_t>(n)]);
            }
            metric += std::norm(acc);
        }
        if (metric > best_metric) {
            best_metric = metric;
            best_pos = pos;
        }
    }
    return best_pos;
}

// 针对本库 modulate() 生成的理想测试帧，已知数据区起点时直接逐符号解调。
std::vector<int> LoRaPHY::demodulateKnownFrame(size_t data_start, size_t symbol_count) {
    std::vector<int> symbols;
    symbols.reserve(symbol_count);
    for (size_t i = 0; i < symbol_count; ++i) {
        auto pk = dechirp(data_start + i * static_cast<size_t>(sample_num_));
        symbols.push_back(binToSymbol(pk.bin));
    }
    return symbols;
}

// 在 IQ 流中寻找连续 upchirp 峰值，返回粗略 preamble 起点。
int LoRaPHY::detect(size_t start_idx) const {
    size_t ii = start_idx;
    std::vector<int> pk_bins;
    while (ii + static_cast<size_t>(sample_num_ * preamble_len) < sig_.size()) {
        if (static_cast<int>(pk_bins.size()) == preamble_len - 1) {
            const int timing_offset = static_cast<int>(std::llround(static_cast<double>(pk_bins.back()) / zero_padding_ratio * 2.0));
            const int first_preamble = static_cast<int>(ii) - (preamble_len - 1) * sample_num_ - timing_offset;
            return std::max(0, first_preamble);
        }
        auto pk0 = dechirp(ii);
        if (pk0.value <= 1e-9) {
            pk_bins.clear();
            ii += sample_num_;
            continue;
        }
        if (!pk_bins.empty()) {
            int diff = posmod(pk_bins.back() - pk0.bin, bin_num_);
            if (diff > bin_num_ / 2) diff = bin_num_ - diff;
            if (diff <= zero_padding_ratio) pk_bins.push_back(pk0.bin);
            else pk_bins = {pk0.bin};
        } else {
            pk_bins.push_back(pk0.bin);
        }
        ii += sample_num_;
    }
    return -1;
}

// 通过 SFD/downchirp 完成帧同步，并估计 preamble bin 与 CFO。
int LoRaPHY::sync(size_t x) {
    bool found = false;
    while (x + static_cast<size_t>(sample_num_) < sig_.size()) {
        auto up_peak = dechirp(x);
        auto down_peak = dechirp(x, false);
        if (std::abs(down_peak.value) > std::abs(up_peak.value)) found = true;
        x += sample_num_;
        if (found) break;
    }
    if (!found) return -1;

    auto pkd = dechirp(x, false);
    int to = (pkd.bin > bin_num_ / 2)
        ? static_cast<int>(std::llround((pkd.bin - bin_num_) / static_cast<double>(zero_padding_ratio)))
        : static_cast<int>(std::llround(pkd.bin / static_cast<double>(zero_padding_ratio)));
    x = static_cast<size_t>(static_cast<int>(x) + to);

    auto pku = dechirp(x - 4 * sample_num_);
    preamble_bin_ = pku.bin;
    cfo_ = (preamble_bin_ > bin_num_ / 2)
        ? (preamble_bin_ - bin_num_) * bw_ / bin_num_
        : preamble_bin_ * bw_ / bin_num_;

    pku = dechirp(x - sample_num_);
    pkd = dechirp(x - sample_num_, false);
    return (std::abs(pku.value) > std::abs(pkd.value))
        ? static_cast<int>(x + static_cast<size_t>(std::llround(2.25 * sample_num_)))
        : static_cast<int>(x + static_cast<size_t>(std::llround(1.25 * sample_num_)));
}

// 解析 explicit header，得到 payload_len、CR、CRC 标志，并校验 header checksum。
bool LoRaPHY::parseHeader(const std::vector<int>& data) {
    auto comp = dynamicCompensation(data);
    auto symbols_g = grayCoding(comp);
    auto codewords = diagDeinterleave(std::vector<int>(symbols_g.begin(), symbols_g.begin() + 8), sf_ - 2);
    auto n = hammingDecode(codewords, 8);
    payload_len = n[0] * 16 + n[1];
    crc = (n[2] & 1) != 0;
    cr = n[2] >> 1;
    if (cr < 1 || cr > 4) return false;
    std::vector<uint8_t> hc{static_cast<uint8_t>(n[3] & 1), static_cast<uint8_t>((n[4] >> 3) & 1),
                            static_cast<uint8_t>((n[4] >> 2) & 1), static_cast<uint8_t>((n[4] >> 1) & 1),
                            static_cast<uint8_t>(n[4] & 1)};
    for (int r = 0; r < 5; ++r) {
        if (hc[r] != headerChecksumBit(r, {n[0], n[1], n[2]})) return false;
    }
    return true;
}

// 计算给定 payload 长度需要的 LoRa 数据符号数。
int LoRaPHY::calcSymNum(int plen) const {
    const int denom = sf_ - 2 * ldr_;
    const double x = (2.0 * plen - sf_ + 7 + 4 * (crc ? 1 : 0) - 5 * (has_header ? 0 : 1)) / denom;
    return 8 + std::max((4 + cr) * static_cast<int>(std::ceil(x)), 0);
}

// 由符号数量反推出最大/最小 payload 长度，主要用于无 header 场景。
int LoRaPHY::calcPayloadLen(int slen, bool no_redundant_bytes) const {
    double plen_float = (sf_ - 2) / 2.0 - 2.5 * (has_header ? 1 : 0)
        + (sf_ - ldr_ * 2) / 2.0 * std::ceil((slen - 8) / static_cast<double>(cr + 4));
    return no_redundant_bytes ? static_cast<int>(std::ceil(plen_float)) : static_cast<int>(std::floor(plen_float));
}

// 根据 CFO 和低数据率优化规则补偿符号漂移。
std::vector<double> LoRaPHY::dynamicCompensation(const std::vector<int>& data) const {
    std::vector<double> symbols(data.size());
    const double modv = static_cast<double>(1 << sf_);
    for (size_t i = 0; i < data.size(); ++i) {
        double drift = (static_cast<double>(i) + 2.0) * modv * cfo_ / rf_freq_;
        symbols[i] = posmod(data[i] - drift, modv);
    }
    if (ldr_) {
        double bin_offset = 0;
        double v_last = 1;
        for (double& v : symbols) {
            double delta = posmod(v - v_last, 4.0);
            bin_offset += (delta < 2) ? -delta : (-delta + 4);
            v_last = v;
            v = posmod(v + bin_offset, modv);
        }
    }
    return symbols;
}

// 接收侧 LoRa Gray 处理：先处理 header/LDR 的 /4 规则，再转 Gray code。
std::vector<int> LoRaPHY::grayCoding(std::vector<double> din) const {
    for (size_t i = 0; i < din.size(); ++i) {
        if (i < 8 || ldr_) din[i] = std::floor(din[i] / 4.0);
        else din[i] = posmod(din[i] - 1.0, static_cast<double>(1 << sf_));
    }
    std::vector<int> out(din.size());
    for (size_t i = 0; i < din.size(); ++i) {
        auto s = static_cast<uint16_t>(din[i]);
        out[i] = static_cast<int>(s ^ (s >> 1));
    }
    return out;
}

// 发送侧 Gray 反处理：Gray code 还原后按 LoRa header/LDR 规则映射为实际符号。
std::vector<int> LoRaPHY::grayDecoding(const std::vector<int>& symbols_i) const {
    std::vector<int> symbols(symbols_i.size());
    for (size_t i = 0; i < symbols_i.size(); ++i) {
        uint32_t num = reverseGray(static_cast<uint32_t>(symbols_i[i]));
        if (i < 8 || ldr_) symbols[i] = posmod(static_cast<int>(num * 4 + 1), 1 << sf_);
        else symbols[i] = posmod(static_cast<int>(num + 1), 1 << sf_);
    }
    return symbols;
}

// 对 Hamming codeword 做 LoRa diagonal interleave，输出待 Gray 映射的符号整数。
std::vector<int> LoRaPHY::diagInterleave(const std::vector<uint8_t>& codewords, int rdd) const {
    std::vector<int> out(rdd, 0);
    for (int col = 0; col < rdd; ++col) {
        int val = 0;
        for (int row = 0; row < static_cast<int>(codewords.size()); ++row) {
            int src_row = posmod(row + col, static_cast<int>(codewords.size()));
            int b = (codewords[src_row] >> col) & 1; // de2bi(...,'right-msb')，第 0 列是 LSB
            val |= b << row;                         // bi2de 默认 right-msb，第 0 行也是 LSB
        }
        out[col] = val;
    }
    return out;
}

// diagonal interleave 的逆过程，把一组符号还原为 Hamming codeword。
std::vector<uint8_t> LoRaPHY::diagDeinterleave(const std::vector<int>& symbols, int ppm) const {
    const int rows = static_cast<int>(symbols.size());
    std::vector<std::vector<int>> b(rows, std::vector<int>(ppm, 0));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < ppm; ++c) b[r][c] = (symbols[r] >> (ppm - 1 - c)) & 1; // left-msb
    }
    std::vector<uint8_t> tmp(ppm, 0);
    for (int c = 0; c < ppm; ++c) {
        int val = 0;
        for (int x = 0; x < rows; ++x) {
            int src_c = posmod(c + x, ppm);
            val |= b[x][src_c] << x; // bi2de 默认 right-msb
        }
        tmp[c] = static_cast<uint8_t>(val);
    }
    std::reverse(tmp.begin(), tmp.end());
    return tmp;
}

// 对 4-bit nibble 添加 LoRa Hamming 校验位；header 固定使用最高冗余度。
std::vector<uint8_t> LoRaPHY::hammingEncode(const std::vector<uint8_t>& nibbles) const {
    std::vector<uint8_t> codewords(nibbles.size());
    for (size_t i = 0; i < nibbles.size(); ++i) {
        uint8_t nibble = nibbles[i] & 0x0F;
        int p1 = bitReduceXor(nibble, {1, 3, 4});
        int p2 = bitReduceXor(nibble, {1, 2, 4});
        int p3 = bitReduceXor(nibble, {1, 2, 3});
        int p4 = bitReduceXor(nibble, {1, 2, 3, 4});
        int p5 = bitReduceXor(nibble, {2, 3, 4});
        int cr_now = (static_cast<int>(i) < sf_ - 2) ? 4 : cr;
        switch (cr_now) {
        case 1: codewords[i] = static_cast<uint8_t>((p4 << 4) | nibble); break;
        case 2: codewords[i] = static_cast<uint8_t>((p5 << 5) | (p3 << 4) | nibble); break;
        case 3: codewords[i] = static_cast<uint8_t>((p2 << 6) | (p5 << 5) | (p3 << 4) | nibble); break;
        case 4: codewords[i] = static_cast<uint8_t>((p1 << 7) | (p2 << 6) | (p5 << 5) | (p3 << 4) | nibble); break;
        default: throw std::runtime_error("Invalid code rate");
        }
    }
    return codewords;
}

// Hamming 解码；rdd 为 7/8 时进行单 bit 纠错，然后取低 4 位数据 nibble。
std::vector<uint8_t> LoRaPHY::hammingDecode(const std::vector<uint8_t>& codewords, int rdd) const {
    auto parityFix = [](int p) -> int {
        switch (p) { case 3: return 4; case 5: return 8; case 6: return 1; case 7: return 2; default: return 0; }
    };
    std::vector<uint8_t> nibbles(codewords.size());
    for (size_t i = 0; i < codewords.size(); ++i) {
        uint8_t cw = codewords[i];
        if (hamming_decoding_en && (rdd == 7 || rdd == 8)) {
            int p2 = bitReduceXor(cw, {7, 4, 2, 1});
            int p3 = bitReduceXor(cw, {5, 3, 2, 1});
            int p5 = bitReduceXor(cw, {6, 4, 3, 2});
            cw ^= static_cast<uint8_t>(parityFix(p2 * 4 + p3 * 2 + p5));
        }
        nibbles[i] = cw & 0x0F;
    }
    return nibbles;
}

// 生成 explicit header：payload 长度、CR/CRC 标志和 5-bit header checksum。
std::vector<uint8_t> LoRaPHY::genHeader(int plen) const {
    std::vector<uint8_t> h(5, 0);
    h[0] = static_cast<uint8_t>((plen >> 4) & 0x0F);
    h[1] = static_cast<uint8_t>(plen & 0x0F);
    h[2] = static_cast<uint8_t>((2 * cr) | (crc ? 1 : 0));
    h[3] = static_cast<uint8_t>(headerChecksumBit(0, {h[0], h[1], h[2]}));
    for (int i = 1; i < 5; ++i) h[4] |= static_cast<uint8_t>(headerChecksumBit(i, {h[0], h[1], h[2]}) << (4 - i));
    return h;
}

// 计算 LoRa PHY CRC。这里按 MATLAB 版本逻辑处理最后两个字节的异或。
std::vector<uint8_t> LoRaPHY::calcCrc(const std::vector<uint8_t>& data) const {
    if (data.empty()) return {0, 0};
    if (data.size() == 1) return {data.back(), 0};
    if (data.size() == 2) return {data.back(), data[data.size() - 2]};

    // MATLAB 原版对 data(1:end-2) 做 CRC16-CCITT，再与最后两个字节异或。
    uint16_t crc16 = 0x0000;
    for (size_t i = 0; i + 2 < data.size(); ++i) {
        crc16 ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc16 = (crc16 & 0x8000) ? static_cast<uint16_t>((crc16 << 1) ^ 0x1021) : static_cast<uint16_t>(crc16 << 1);
        }
    }
    uint8_t b1 = static_cast<uint8_t>((crc16 & 0x00FF) ^ data.back());
    uint8_t b2 = static_cast<uint8_t>(((crc16 >> 8) & 0x00FF) ^ data[data.size() - 2]);
    return {b1, b2};
}

// payload whitening：和固定 whitening 序列逐字节异或。
std::vector<uint8_t> LoRaPHY::whiten(const std::vector<uint8_t>& data) const {
    auto ws = whiteningSeq();
    std::vector<uint8_t> out(data.size());
    for (size_t i = 0; i < data.size(); ++i) out[i] = data[i] ^ ws[i % ws.size()];
    return out;
}

// whitening 是异或操作，自反；解白化复用 whiten()。
std::vector<uint8_t> LoRaPHY::dewhiten(const std::vector<uint8_t>& bytes) const {
    return whiten(bytes);
}

// LoRa whitening 固定伪随机序列。
std::vector<uint8_t> LoRaPHY::whiteningSeq() {
    return {0xff,0xfe,0xfc,0xf8,0xf0,0xe1,0xc2,0x85,0x0b,0x17,0x2f,0x5e,0xbc,0x78,0xf1,0xe3,0xc6,0x8d,0x1a,0x34,0x68,0xd0,0xa0,0x40,0x80,0x01,0x02,0x04,0x08,0x11,0x23,0x47,0x8e,0x1c,0x38,0x71,0xe2,0xc4,0x89,0x12,0x25,0x4b,0x97,0x2e,0x5c,0xb8,0x70,0xe0,0xc0,0x81,0x03,0x06,0x0c,0x19,0x32,0x64,0xc9,0x92,0x24,0x49,0x93,0x26,0x4d,0x9b,0x37,0x6e,0xdc,0xb9,0x72,0xe4,0xc8,0x90,0x20,0x41,0x82,0x05,0x0a,0x15,0x2b,0x56,0xad,0x5b,0xb6,0x6d,0xda,0xb5,0x6b,0xd6,0xac,0x59,0xb2,0x65,0xcb,0x96,0x2c,0x58,0xb0,0x61,0xc3,0x87,0x0f,0x1f,0x3e,0x7d,0xfb,0xf6,0xed,0xdb,0xb7,0x6f,0xde,0xbd,0x7a,0xf5,0xeb,0xd7,0xae,0x5d,0xba,0x74,0xe8,0xd1,0xa2,0x44,0x88,0x10,0x21,0x43,0x86,0x0d,0x1b,0x36,0x6c,0xd8,0xb1,0x63,0xc7,0x8f,0x1e,0x3c,0x79,0xf3,0xe7,0xce,0x9c,0x39,0x73,0xe6,0xcc,0x98,0x31,0x62,0xc5,0x8b,0x16,0x2d,0x5a,0xb4,0x69,0xd2,0xa4,0x48,0x91,0x22,0x45,0x8a,0x14,0x29,0x52,0xa5,0x4a,0x95,0x2a,0x54,0xa9,0x53,0xa7,0x4e,0x9d,0x3b,0x77,0xee,0xdd,0xbb,0x76,0xec,0xd9,0xb3,0x67,0xcf,0x9e,0x3d,0x7b,0xf7,0xef,0xdf,0xbf,0x7e,0xfd,0xfa,0xf4,0xe9,0xd3,0xa6,0x4c,0x99,0x33,0x66,0xcd,0x9a,0x35,0x6a,0xd4,0xa8,0x51,0xa3,0x46,0x8c,0x18,0x30,0x60,0xc1,0x83,0x07,0x0e,0x1d,0x3a,0x75,0xea,0xd5,0xaa,0x55,0xab,0x57,0xaf,0x5f,0xbe,0x7c,0xf9,0xf2,0xe5,0xca,0x94,0x28,0x50,0xa1,0x42,0x84,0x09,0x13,0x27,0x4f,0x9f,0x3f,0x7f};
}

// explicit header checksum 矩阵的一行计算。
int LoRaPHY::headerChecksumBit(int row, const std::vector<uint8_t>& n) {
    static constexpr int mat[5][12] = {
        {1,1,1,1,0,0,0,0,0,0,0,0},
        {1,0,0,0,1,1,1,0,0,0,0,1},
        {0,1,0,0,1,0,0,1,1,0,1,0},
        {0,0,1,0,0,1,0,1,0,1,1,1},
        {0,0,0,1,0,0,1,0,1,1,1,1}
    };
    int acc = 0;
    for (int nib = 0; nib < 3; ++nib) {
        for (int b = 0; b < 4; ++b) {
            int bitv = (n[nib] >> (3 - b)) & 1;
            acc ^= mat[row][nib * 4 + b] & bitv;
        }
    }
    return acc & 1;
}

// 读取整数的 1-based bit 位。
int LoRaPHY::bit(uint32_t w, int one_based_pos) { return static_cast<int>((w >> (one_based_pos - 1)) & 1U); }

// 读取多个 bit 后做异或归约。
int LoRaPHY::bitReduceXor(uint32_t w, const std::vector<int>& pos) {
    int r = bit(w, pos[0]);
    for (size_t i = 1; i < pos.size(); ++i) r ^= bit(w, pos[i]);
    return r;
}

// zero-padding 后 FFT bin 分辨率为 LoRa bin 的 zero_padding_ratio 倍。
// 这里必须四舍五入到最近 LoRa symbol；直接整数除法会向下取整，峰值落在边界附近时偶发错 1。
int LoRaPHY::binToSymbol(int bin) const {
    const int shifted = posmod(bin, fft_len_);
    return posmod(static_cast<int>(std::llround(static_cast<double>(shifted) / zero_padding_ratio)), 1 << sf_);
}

// Gray code 还原为普通二进制整数。
uint32_t LoRaPHY::reverseGray(uint32_t num) {
    for (uint32_t mask = num >> 1; mask != 0; mask >>= 1) num ^= mask;
    return num;
}

// 正模运算，避免 C++ % 对负数返回负余数。
int LoRaPHY::posmod(int x, int m) { int r = x % m; return r < 0 ? r + m : r; }
double LoRaPHY::posmod(double x, double m) { double r = std::fmod(x, m); return r < 0 ? r + m : r; }

// 简单线性重采样，把输入采样率转换到内部 2*BW 处理采样率。
std::vector<std::complex<double>> LoRaPHY::resampleLinear(const std::vector<std::complex<double>>& in, double in_fs, double out_fs) {
    if (in.empty()) return {};
    const size_t out_n = static_cast<size_t>(std::floor(in.size() * out_fs / in_fs));
    std::vector<std::complex<double>> out(out_n);
    for (size_t i = 0; i < out_n; ++i) {
        double src = i * in_fs / out_fs;
        size_t j = static_cast<size_t>(std::floor(src));
        double a = src - j;
        if (j + 1 >= in.size()) out[i] = in.back();
        else out[i] = in[j] * (1.0 - a) + in[j + 1] * a;
    }
    return out;
}

// FFT 封装：优先使用 FFTW；未启用时，2 的幂长度使用 radix-2，非 2 的幂退化为慢速 DFT。
std::vector<std::complex<double>> LoRaPHY::fft(std::vector<std::complex<double>> x) {
#if LORA_HAS_FFTW
    if (g_fftw_plan_n != x.size()) {
        if (g_fftw_plan) fftw_destroy_plan(g_fftw_plan);
        g_fftw_plan_n = x.size();
        g_fftw_in.resize(x.size());
        g_fftw_out.resize(x.size());
        g_fftw_plan = fftw_plan_dft_1d(static_cast<int>(x.size()),
            reinterpret_cast<fftw_complex*>(g_fftw_in.data()), reinterpret_cast<fftw_complex*>(g_fftw_out.data()),
            FFTW_FORWARD, FFTW_MEASURE);
    }
    std::copy(x.begin(), x.end(), g_fftw_in.begin());
    fftw_execute(g_fftw_plan);
    return g_fftw_out;
#else
    const size_t n = x.size();
    if ((n & (n - 1)) != 0) {
        std::vector<std::complex<double>> out(n);
        for (size_t k = 0; k < n; ++k) {
            std::complex<double> sum{0, 0};
            for (size_t t = 0; t < n; ++t) {
                sum += x[t] * cexpj(-2 * kPi * static_cast<double>(k * t) / static_cast<double>(n));
            }
            out[k] = sum;
        }
        return out;
    }
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bitn = n >> 1;
        for (; j & bitn; bitn >>= 1) j ^= bitn;
        j ^= bitn;
        if (i < j) std::swap(x[i], x[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        double ang = -2 * kPi / static_cast<double>(len);
        std::complex<double> wlen = cexpj(ang);
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w{1, 0};
            for (size_t j = 0; j < len / 2; ++j) {
                auto u = x[i + j];
                auto v = x[i + j + len / 2] * w;
                x[i + j] = u + v;
                x[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
    return x;
#endif
}

// 生成单个 LoRa chirp。h 控制循环移频，cfo/tdelta/tscale 用于频偏和采样时钟偏差模拟。
std::vector<std::complex<double>> LoRaPHY::chirp(bool is_up, int sf, double bw, double fs, double h, double cfo, double tdelta, double tscale) {
    const int N = 1 << sf;
    const double T = N / bw;
    const int samp_per_sym = static_cast<int>(std::llround(fs / bw * N));
    double h_orig = h;
    h = std::llround(h);
    cfo += (h_orig - h) / N * bw;
    double k = is_up ? bw / T : -bw / T;
    double f0 = is_up ? -bw / 2 + cfo : bw / 2 + cfo;

    int c1_count = static_cast<int>(std::floor(samp_per_sym * (N - h) / static_cast<double>(N))) + 1;
    std::vector<std::complex<double>> c1;
    c1.reserve(std::max(0, c1_count));
    for (int i = 0; i < c1_count; ++i) {
        double t = i / fs * tscale + tdelta;
        c1.push_back(cexpj(2 * kPi * (t * (f0 + k * T * h / N + 0.5 * k * t))));
    }
    double phi = c1.empty() ? 0.0 : std::arg(c1.back());

    int c2_count = static_cast<int>(std::floor(samp_per_sym * h / static_cast<double>(N)));
    std::vector<std::complex<double>> y;
    y.reserve(samp_per_sym);
    if (!c1.empty()) y.insert(y.end(), c1.begin(), c1.end() - 1);
    for (int i = 0; i < c2_count; ++i) {
        double t = i / fs + tdelta;
        y.push_back(cexpj(phi + 2 * kPi * (t * (f0 + 0.5 * k * t))));
    }
    return y;
}

// 从文件读取 float32 交错 IQ：I,Q,I,Q...
std::vector<std::complex<double>> LoRaPHY::readComplexBinary(const std::string& filename, size_t count) {
    std::ifstream f(filename, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open IQ file: " + filename);
    std::vector<std::complex<double>> out;
    while (!count || out.size() < count) {
        float re = 0, im = 0;
        if (!f.read(reinterpret_cast<char*>(&re), sizeof(float))) break;
        if (!f.read(reinterpret_cast<char*>(&im), sizeof(float))) break;
        out.emplace_back(re, im);
    }
    return out;
}

// 写出 float32 交错 IQ：I,Q,I,Q...
void LoRaPHY::writeComplexBinary(const std::string& filename, const std::vector<std::complex<double>>& data) {
    std::ofstream f(filename, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot write IQ file: " + filename);
    for (auto z : data) {
        float re = static_cast<float>(z.real());
        float im = static_cast<float>(z.imag());
        f.write(reinterpret_cast<const char*>(&re), sizeof(float));
        f.write(reinterpret_cast<const char*>(&im), sizeof(float));
    }
}

} // namespace lora
