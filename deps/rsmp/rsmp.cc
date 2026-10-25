#include "rsmp.hh"

#include <CDSPResampler.h>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rsmp {

namespace {

/// Filter shape: transition band in percent of the space between 0 and the
/// output Nyquist, and the required stopband attenuation in dB. Narrower and
/// deeper means more latency.
constexpr Number trans_band = 10.0;
constexpr Number attenuation = 90.0;
constexpr std::size_t output_capacity_margin = 2;

std::size_t output_capacity(const Config& config) {
    return static_cast<std::size_t>(
        config.block_frames * config.output_rate / config.input_rate
    ) + output_capacity_margin;
}

Config checked(Config config) {
    if (config.input_rate <= 0.0 || config.output_rate <= 0.0) {
        throw std::invalid_argument("rsmp: rates must be positive");
    }
    if (config.block_frames < 1) {
        throw std::invalid_argument("rsmp: block_frames must be positive");
    }
    return config;
}

} // namespace

class ResamplerImpl {
public:
    explicit ResamplerImpl(Config config);

    std::optional<Sample> push(Sample sample);

private:
    Config _config;
    r8b::CDSPResampler _left;
    r8b::CDSPResampler _right;

    std::vector<Number> _input_left;
    std::vector<Number> _input_right;
    std::size_t _ready = 0;

    std::vector<Number> _output_left;
    std::vector<Number> _output_right;
    std::size_t _head = 0;
    std::size_t _total = 0;

    void flush_block();
};

ResamplerImpl::ResamplerImpl(Config config)
    : _config(checked(std::move(config)))
    , _left(
        _config.input_rate,
        _config.output_rate,
        _config.block_frames,
        trans_band,
        attenuation
    )
    , _right(
        _config.input_rate,
        _config.output_rate,
        _config.block_frames,
        trans_band,
        attenuation
    )
{
    _input_left.resize(static_cast<std::size_t>(_config.block_frames));
    _input_right.resize(static_cast<std::size_t>(_config.block_frames));
    _output_left.resize(output_capacity(_config));
    _output_right.resize(output_capacity(_config));
}

void ResamplerImpl::flush_block() {
    Number* out = nullptr;
    const int produced = _left.process(_input_left.data(), _config.block_frames, out);
    if (produced < 0 || static_cast<std::size_t>(produced) > _output_left.size()) {
        throw std::runtime_error("rsmp: output block does not fit the buffer");
    }
    std::copy_n(out, produced, _output_left.begin());

    out = nullptr;
    _right.process(_input_right.data(), _config.block_frames, out);
    std::copy_n(out, produced, _output_right.begin());

    _head = 0;
    _total = static_cast<std::size_t>(produced);
    _ready = 0;
}

std::optional<Sample> ResamplerImpl::push(Sample sample) {
    _input_left[_ready] = sample.left;
    _input_right[_ready] = sample.right;
    if (++_ready == static_cast<std::size_t>(_config.block_frames)) {
        flush_block();
    }

    if (_total == 0) {
        return std::nullopt;
    }
    const Sample out { _output_left[_head], _output_right[_head] };
    if (++_head >= _total) {
        _total = 0;
    }
    return out;
}

Resampler::Resampler(Config config)
    : _impl(std::make_unique<ResamplerImpl>(std::move(config))) {
}

Resampler::~Resampler() = default;

std::optional<Sample> Resampler::push(Sample sample) {
    return _impl->push(sample);
}

} // namespace rsmp
