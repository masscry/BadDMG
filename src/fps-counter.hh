#pragma once

#include <bdmg/common.hh>

#include <chrono>

namespace bdmg {

namespace time = std::chrono;

class FpsCounter {
public:
    void update();
    auto get() const;
private:
    time::steady_clock::time_point _lastUpdate = time::steady_clock::now();
    unsigned int _frames = 0;
    float _fps = 0.0f;
};

inline
void FpsCounter::update() {
    auto now = time::steady_clock::now();
    _frames++;
    
    auto elapsed = time::duration_cast<time::seconds>(now - _lastUpdate).count();
    if (elapsed >= 1) {
        _fps = static_cast<float>(_frames) / elapsed;
        _frames = 0;
        _lastUpdate = now;
    }
}

inline
auto FpsCounter::get() const {
    return _fps;
}

} // namespace bdmg
