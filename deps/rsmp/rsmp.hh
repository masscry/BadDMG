#pragma once

#include <memory>
#include <optional>

namespace rsmp {

/// Samples are doubles here: the maths inside is double precision, and callers
/// narrow to whatever their own audio path uses on the way out.
using Number = double;

/// Input samples gathered per library call: large enough to amortize the
/// call, small enough not to add latency of its own.
constexpr int default_block_frames = 4096;

struct Config {
    Number input_rate;
    Number output_rate;
    int block_frames = default_block_frames;
};

struct Sample {
    Number left;
    Number right;
};

class ResamplerImpl;

class Resampler {
public:
    explicit Resampler(Config);
    Resampler(const Resampler&) = delete;
    Resampler& operator=(const Resampler&) = delete;
    ~Resampler();

    std::optional<Sample> push(Sample);

private:
    std::unique_ptr<ResamplerImpl> _impl;
};

} // namespace rsmp
