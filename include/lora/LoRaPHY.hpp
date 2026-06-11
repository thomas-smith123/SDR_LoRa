#pragma once

#include <complex>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace lora {

// 解调得到的一个 LoRa 包：包含物理层符号、频偏估计、NETID，以及点级对齐后的整帧 IQ。
struct DemodPacket {
    std::vector<int> symbols;
    double cfo_hz = 0.0;
    std::pair<int, int> net_id{0, 0};
    size_t frame_start_sample = 0; // 整帧起点；refineAlignedFrame() 后为原始输入采样率下的点级对齐起点
    size_t data_start_sample = 0;  // payload 第一个 data chirp 起点；refineAlignedFrame() 后为原始输入采样率下标
    size_t frame_end_sample = 0;   // 整帧结束位置，左闭右开；refineAlignedFrame() 后为原始输入采样率下标
    std::vector<std::complex<double>> aligned_iq; // CRC 通过后填充的原始输入采样率完整一帧 IQ
};

// 解码结果：data 为 payload；若 CRC 开启，末尾会附带收到的 2 字节 CRC，checksum 为重新计算的 CRC。
struct DecodeResult {
    std::vector<uint8_t> data;
    std::vector<uint8_t> checksum;
    bool crc_ok = true; // crc=false 时保持 true；crc=true 时表示收到的 CRC 是否匹配重新计算结果
};

// LoRa 物理层编解码器。
// 当前实现面向基带复数 IQ：支持 payload 编码、LoRa chirp 调制、包检测/同步、符号解调和 payload 解码。
class LoRaPHY {
public:
    // rf_freq: 射频中心频率；sf: 扩频因子；bw: LoRa 带宽；fs: 输入/输出 IQ 采样率；preamble_len: 前导 upchirp 数量，默认 8。
    LoRaPHY(double rf_freq, int sf, double bw, double fs, int preamble_len = 8);

    int cr = 1;                 // 1:4/5, 2:4/6, 3:4/7, 4:4/8
    int payload_len = 0;        // 显式头模式下由 header 解出；隐式头模式下需由外部设置
    bool has_header = true;     // true: explicit header；false: implicit header
    bool crc = true;            // 是否在 payload 后附带 LoRa PHY CRC
    int preamble_len = 8;       // 前导 upchirp 数量，LoRaWAN 常见默认值为 8
    bool fast_mode = true;      // C++ 版默认跳过低通滤波；输入最好为基带 IQ
    bool hamming_decoding_en = true; // Hamming(7/8) 纠错开关
    bool is_debug = false;      // 预留调试开关
    bool ideal_fallback_enabled = false; // 仅用于本库自发自收测试；真实 SDR 接收应关闭，避免噪声窗口被当成理想帧解码
    int zero_padding_ratio = 8;  // 8 可让 2*2^SF*ratio 成为 2 的幂，内置 FFT 速度快很多

    // 根据当前参数重新计算内部派生量，并重新生成上下 chirp 模板；修改 cr/sf/bw/fs/zero_padding_ratio 后应调用。
    void init();

    // 将 payload 字节编码为 LoRa 物理层符号：CRC -> whitening -> header -> Hamming -> diagonal interleave -> Gray 映射。
    std::vector<int> encode(const std::vector<uint8_t>& payload);

    // 将 LoRa 符号调制成复数 IQ：preamble -> NETID chirp -> SFD -> data chirps。
    std::vector<std::complex<double>> modulate(const std::vector<int>& symbols) const;

    // 从复数 IQ 中检测 LoRa 包并解调成符号；真实 SDR 数据应使用此入口。
    std::vector<DemodPacket> demodulate(const std::vector<std::complex<double>>& input);

    // 将解调得到的符号还原为 payload 字节，并解析 header/CRC。
    DecodeResult decode(const std::vector<int>& symbols);

    // CRC 通过后再调用：对指定包做点级 preamble 精同步，并截取完整一帧 IQ。
    bool refineAlignedFrame(DemodPacket& packet) const;

    // 批量 decode demodulate() 返回的多个包。
    std::vector<DecodeResult> decodePackets(const std::vector<DemodPacket>& packets);

    // 按当前参数估计 payload 空口时间，单位 ms。
    double timeOnAirMs(int payload_length) const;

    // 读取 interleaved float32 IQ 二进制文件：float I, float Q, float I, float Q...
    static std::vector<std::complex<double>> readComplexBinary(const std::string& filename, size_t count = 0);

    // 写出 interleaved float32 IQ 二进制文件。
    static void writeComplexBinary(const std::string& filename, const std::vector<std::complex<double>>& data);

    // 生成一个 LoRa upchirp/downchirp；h 为符号偏移，cfo 为频偏，tdelta/tscale 用于时间偏移/缩放。
    static std::vector<std::complex<double>> chirp(bool is_up, int sf, double bw, double fs,
                                                    double h = 0.0, double cfo = 0.0,
                                                    double tdelta = 0.0, double tscale = 1.0);

private:
    struct Peak { double value = 0.0; int bin = 0; }; // bin 为 0-based

    double rf_freq_;
    int sf_;
    double bw_;
    double fs_;
    int ldr_ = 0;
    int sample_num_ = 0;
    int bin_num_ = 0;
    int fft_len_ = 0;
    int preamble_bin_ = 0;
    double cfo_ = 0.0;
    std::vector<std::complex<double>> sig_;
    std::vector<std::complex<double>> raw_sig_;
    std::vector<std::complex<double>> downchirp_;
    std::vector<std::complex<double>> upchirp_;

    Peak dechirp(size_t start, bool is_up = true) const; // 与本地下/上 chirp 相乘后做 FFT，找最大频率 bin
    int refinePreambleStart(size_t coarse_start) const; // 在粗检测点附近逐采样点相关搜索，得到点级 preamble 起点
    std::vector<int> demodulateKnownFrame(size_t data_start, size_t symbol_count); // 理想自生成帧的兜底解调路径
    int detect(size_t start_idx) const; // 通过连续 upchirp 峰值检测 preamble 起点
    int sync(size_t x); // 根据 downchirp/SFD 细同步，并估计 CFO
    bool parseHeader(const std::vector<int>& data); // 只解析前 8 个符号中的 explicit header
    int calcSymNum(int plen) const; // 根据 payload 长度、SF、CR、CRC、header 计算数据符号数
    int calcPayloadLen(int slen, bool no_redundant_bytes) const; // 根据符号数反推 payload 长度
    std::vector<double> dynamicCompensation(const std::vector<int>& data) const; // CFO/LDR 符号漂移补偿
    std::vector<int> grayCoding(std::vector<double> din) const; // LoRa 解调侧 Gray 编码变换
    std::vector<int> grayDecoding(const std::vector<int>& symbols_i) const; // LoRa 调制侧 Gray 反变换
    std::vector<int> diagInterleave(const std::vector<uint8_t>& codewords, int rdd) const; // LoRa diagonal interleave
    std::vector<uint8_t> diagDeinterleave(const std::vector<int>& symbols, int ppm) const; // LoRa diagonal deinterleave
    std::vector<uint8_t> hammingEncode(const std::vector<uint8_t>& nibbles) const; // 4-bit nibble 加 Hamming 校验位
    std::vector<uint8_t> hammingDecode(const std::vector<uint8_t>& codewords, int rdd) const; // Hamming 纠错并取低 4 位 nibble
    std::vector<uint8_t> genHeader(int plen) const; // 生成 explicit header 的 5 个 nibble
    std::vector<uint8_t> calcCrc(const std::vector<uint8_t>& data) const; // LoRa PHY CRC 计算
    std::vector<uint8_t> whiten(const std::vector<uint8_t>& data) const; // payload whitening
    std::vector<uint8_t> dewhiten(const std::vector<uint8_t>& bytes) const; // whitening 自反，解白化同样异或序列

    static std::vector<uint8_t> whiteningSeq(); // 固定 whitening 序列
    static int headerChecksumBit(int row, const std::vector<uint8_t>& first_three_nibbles); // header 5-bit checksum
    static int bit(uint32_t w, int one_based_pos); // 读取 1-based bit
    static int bitReduceXor(uint32_t w, const std::vector<int>& pos); // 多个 bit 异或
    static uint32_t reverseGray(uint32_t num); // Gray code -> binary
    static int posmod(int x, int m); // 正模，保证结果在 [0,m)
    static double posmod(double x, double m); // double 版本正模
    int binToSymbol(int bin) const; // 将 zero-padding FFT bin 四舍五入量化到 LoRa symbol
    static std::vector<std::complex<double>> resampleLinear(const std::vector<std::complex<double>>& in, double in_fs, double out_fs); // 线性重采样到 2*BW
    static std::vector<std::complex<double>> fft(std::vector<std::complex<double>> x); // FFTW 或内置 radix-2 FFT
};

} // namespace lora
