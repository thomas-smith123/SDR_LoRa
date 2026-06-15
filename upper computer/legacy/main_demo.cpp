#include "LoRaPHY.hpp"

#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

int main()
{
	try {
		// 典型 LoRa 参数：433 MHz / SF7 / BW125k / 采样率 1 MSps。
		// 如果有真实 IQ 文件，可用 LoRaPHY::readComplexBinary("xxx.bin") 读入。
		lora::LoRaPHY phy(433e6, 7, 125e3, 1e6, 8);
		phy.cr = 1;
		phy.has_header = true;
		phy.crc = true;
		phy.fast_mode = true;
		phy.zero_padding_ratio = 8;
		phy.init();

		std::vector<uint8_t> payload = {'H', 'E', 'L', 'L', 'O'};
		auto symbols = phy.encode(payload);
		auto iq = phy.modulate(symbols);

		std::mt19937 rng(std::random_device{}());
		std::uniform_int_distribution<size_t> padding_dist(0, 5000);
		const size_t padding_len = padding_dist(rng);
		std::vector<std::complex<double>> rx(padding_len, {0.0, 0.0});
		rx.insert(rx.end(), iq.begin(), iq.end());

		auto packets = phy.demodulate(rx);

		std::cout << "Encoded symbols: ";
		for (int s : symbols) {
			std::cout << s << ' ';
		}
		std::cout << "\nGenerated IQ samples: " << iq.size() << "\n";
		std::cout << "Random front padding samples: " << padding_len << "\n";
		std::cout << "RX IQ samples with padding: " << rx.size() << "\n";
		std::cout << "Detected packets: " << packets.size() << "\n";

		for (size_t i = 0; i < packets.size(); ++i) {
			auto decoded = phy.decode(packets[i].symbols);
			std::cout << "Packet #" << i + 1 << " CFO=" << packets[i].cfo_hz << " Hz, NETID=("
				<< packets[i].net_id.first << ", " << packets[i].net_id.second << ")\n";
			std::cout << "Decoded bytes(hex): ";
			for (uint8_t b : decoded.data) {
				std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
					<< static_cast<int>(b) << ' ';
			}
			std::cout << std::dec << "\nCRC: " << (decoded.crc_ok ? "OK" : "FAIL") << "\n";
			std::cout << "Decoded text: ";
			for (size_t k = 0; k < payload.size() && k < decoded.data.size(); ++k) {
				std::cout << static_cast<char>(decoded.data[k]);
			}
			std::cout << "\n";
			if (decoded.crc_ok) {
				if (!phy.refineAlignedFrame(packets[i])) {
					std::cout << "Aligned frame refine failed\n";
					continue;
				}
				const long long start_error = static_cast<long long>(packets[i].frame_start_sample) - static_cast<long long>(padding_len);
				std::cout << "Expected frame start: " << padding_len
					<< ", detected frame start: " << packets[i].frame_start_sample
					<< ", error: " << start_error << " samples\n";
				std::cout << "Aligned frame samples: [" << packets[i].frame_start_sample << ", "
					<< packets[i].frame_end_sample << "), length=" << packets[i].aligned_iq.size() << "\n";
				const std::string frame_file = "aligned_frame_" + std::to_string(i + 1) + ".bin";
				lora::LoRaPHY::writeComplexBinary(frame_file, packets[i].aligned_iq);
				std::cout << "Saved aligned frame IQ: " << frame_file << "\n";
			}
		}
	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << "\n";
		return 1;
	}

	return 0;
}