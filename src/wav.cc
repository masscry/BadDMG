#include "wav.hh"

#include <cstring>
#include <filesystem>
#include <format>
#include <ios>
#include <stdexcept>

namespace bdmg {

namespace {

static_assert(sizeof(float) == 4,
    "the WAV header declares 32-bit float samples; push() writes sizeof(float) per channel");

// All supported hosts are little-endian.
template <typename T>
void put_le(char* dst, T value) {
    std::memcpy(dst, &value, sizeof(value));
}

} // namespace

WavWriter::WavWriter() = default;

WavWriter::~WavWriter() {
    close();
}

void WavWriter::open(const std::filesystem::path& path, u32 sample_rate) {
    close();
    _file.open(path, std::ios::binary);
    if (!_file) {
        throw std::runtime_error(
            std::format("failed to open WAV file ({})", path.native()));
    }
    _sample_rate = sample_rate;
    _frames = 0;
    write_header(0);
}

void WavWriter::push(const AudioSample& sample) {
    if (!_file.is_open()) {
        return;
    }
    const float left = static_cast<float>(sample.left);
    const float right = static_cast<float>(sample.right);
    _file.write(reinterpret_cast<const char*>(&left), sizeof(left));
    _file.write(reinterpret_cast<const char*>(&right), sizeof(right));
    ++_frames;
}

void WavWriter::close() {
    if (!_file.is_open()) {
        return;
    }
    _file.flush();
    _file.seekp(0);
    write_header(_frames);
    _file.close();
}

void WavWriter::write_header(u64 frames) {
    constexpr u16 format_f32 = 3;
    constexpr u16 channels = 2;
    constexpr u16 bits_per_sample = 32;
    const u32 block_align = channels * bits_per_sample / 8;
    const u32 byte_rate = _sample_rate * block_align;
    const u32 data_size = static_cast<u32>(frames * block_align);

    char header[44] = {};
    std::memcpy(header + 0, "RIFF", 4);
    put_le(header + 4, 36u + data_size);
    std::memcpy(header + 8, "WAVE", 4);
    std::memcpy(header + 12, "fmt ", 4);
    put_le(header + 16, u32{16});
    put_le(header + 20, format_f32);
    put_le(header + 22, channels);
    put_le(header + 24, _sample_rate);
    put_le(header + 28, byte_rate);
    put_le(header + 32, block_align);
    put_le(header + 34, bits_per_sample);
    std::memcpy(header + 36, "data", 4);
    put_le(header + 40, data_size);
    _file.write(header, sizeof(header));
}

} // namespace bdmg
