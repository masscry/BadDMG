#pragma once

#include "sdl.hh"

#include <bdmg/common.hh>

#include <memory>

namespace bdmg {

// Mirrors the motor of a rumble cartridge onto the device vibrator.
//
// On Android 14+ the pulses go through MainActivity.rumblePulse(), which uses
// the physical emulation vibration usage. Otherwise (older Android, desktop)
// it opens the first SDL haptic device that supports rumble (on Android this
// is the phone's vibrator). Without any of them -- or without the haptic
// subsystem -- the object stays inert and update() does nothing.
class Rumble {
public:
    Rumble();

    // Call once per emulated frame with the cartridge motor state and a
    // monotonic timestamp in nanoseconds.
    void update(bool motor_on, u64 now_ns);

private:
    bool pulse();
    void stop();

    Haptic _haptic;
    // Android only: false once MainActivity reported it cannot vibrate.
    bool _native { true };
    bool _on { false };
    u64 _last_pulse_ns { 0 };
};

} // namespace bdmg
