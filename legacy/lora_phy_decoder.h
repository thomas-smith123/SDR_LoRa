#ifndef LORA_PHY_DECODER_H
#define LORA_PHY_DECODER_H

#include <complex>
#include <cstdint>
#include <string>
#include <vector>

class LoRaPhyDecoder
{
public:
    struct Config {
        double rfFreqHz = 433e6;
        int sf = 7;
        double bandwidthHz = 125e3;
        double sampleRateHz = 250e3;
        int cr = 1;
        int payloadLen = 0;
        bool hasHeader = true;
        bool crc = true;
        bool lowDataRateOptimize = false;
        int preambleLen = 6;
        int zeroPaddingRatio = 10;
    };

    struct Packet {
        std::vector<int> symbols;
        std::vector<uint8_t> payload;
        std::vector<uint8_t> decodedCrc;
        std::vector<uint8_t> calculatedCrc;
        double cfoHz = 0.0;
        int netId0 = -1;
        int netId1 = -1;
        bool headerValid = false;
        bool crcValid = false;
        std::string error;
    };

    LoRaPhyDecoder();
    explicit LoRaPhyDecoder(Config config);

    void setConfig(const Config& config);
    const Config& config() const { return cfg_; }

    std::vector<Packet> demodulate(const std::vector<std::complex<float>>& iq);
    Packet decodeSymbols(const std::vector<int>& symbols);

private:
    Config cfg_;
    int symbolCount_ = 0;
    int sampleNum_ = 0;
    int binNum_ = 0;
    int fftLen_ = 0;
    double cfoHz_ = 0.0;
    int preambleBin_ = 0;
    std::vector<std::complex<double>> upchirp_;
    std::vector<std::complex<double>> downchirp_;

    void init();
    std::vector<std::complex<double>> chirp(bool isUp, double h, double cfoHz, double tdelta = 0.0) const;
    std::pair<double, int> dechirp(const std::vector<std::complex<double>>& sig, int start, bool upSymbol = true) const;
    int detect(const std::vector<std::complex<double>>& sig, int start) const;
    int sync(const std::vector<std::complex<double>>& sig, int start);
    bool parseHeader(const std::vector<int>& symbols, Packet* packet = nullptr);
    int calcSymbolNum(int payloadLen) const;

    std::vector<int> dynamicCompensation(const std::vector<int>& symbols) const;
    std::vector<uint16_t> grayCoding(std::vector<int> symbols) const;
    std::vector<uint16_t> diagDeinterleave(const std::vector<uint16_t>& symbols, int ppm) const;
    std::vector<uint8_t> hammingDecode(const std::vector<uint16_t>& codewords, int rdd) const;
    std::vector<uint8_t> dewhiten(const std::vector<uint8_t>& bytes) const;
    std::vector<uint8_t> calcCrc(const std::vector<uint8_t>& data) const;

    static int bit(uint16_t word, int matlabOneBasedPos);
    static uint16_t circShiftRight(uint16_t bits, int width, int shift);
    static uint8_t parityFix(int p);
};

#endif // LORA_PHY_DECODER_H
