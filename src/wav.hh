#pragma once

#include <bdmg/audio.hh>
#include <bdmg/common.hh>

#include <filesystem>
#include <fstream>

namespace bdmg {

class WavWriter {
public:
    WavWriter();
    ~WavWriter();

    WavWriter(const WavWriter&) = delete;
    WavWriter& operator=(const WavWriter&) = delete;

    void open(const std::filesystem::path& path, u32 sample_rate);
    void push(const AudioSample& sample);
    void close();

private:
    void write_header(u64 frames);

    std::ofstream _file;
    u32 _sample_rate = 0;
    u64 _frames = 0;
};

} // namespace bdmg
