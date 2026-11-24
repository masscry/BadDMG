#include "rumble.hh"

#include <SDL3/SDL.h>
#include <SDL3/SDL_haptic.h>

#include <cstring>

#ifdef __ANDROID__
#include <SDL3/SDL_system.h>
#include <jni.h>
#endif

namespace bdmg {

namespace {

// Android vibrates for a fixed duration, so a running motor is expressed as
// short pulses that are renewed before they end. A pulse that is not renewed
// (the game switched the motor off, or the app paused) dies on its own.
constexpr Uint32 pulse_ms = 100;
constexpr u64 renew_ns = 70'000'000;
constexpr float strength = 1.0f;

#ifdef __ANDROID__
// Calls a method of the running MainActivity. Returns false if it could not be
// called or, for pulses, if the activity reported it cannot vibrate.
bool call_activity(const char* name, const char* sig, bool* result, jint arg) {
    auto* env = static_cast<JNIEnv*>(SDL_GetAndroidJNIEnv());
    auto activity = static_cast<jobject>(SDL_GetAndroidActivity());
    if (!env || !activity) {
        return false;
    }
    bool ok = false;
    if (jclass cls = env->GetObjectClass(activity)) {
        if (jmethodID mid = env->GetMethodID(cls, name, sig)) {
            if (sig[std::strlen(sig) - 1] == 'Z') {
                *result = env->CallBooleanMethod(activity, mid, arg) == JNI_TRUE;
            } else {
                env->CallVoidMethod(activity, mid);
                *result = true;
            }
            ok = true;
        }
        env->DeleteLocalRef(cls);
    }
    env->DeleteLocalRef(activity);
    if (env->ExceptionCheck()) {
        env->ExceptionClear();
        return false;
    }
    return ok;
}
#endif

} // namespace

bool Rumble::pulse() {
#ifdef __ANDROID__
    if (_native) {
        bool vibrated = false;
        if (call_activity("rumblePulse", "(I)Z", &vibrated, static_cast<jint>(pulse_ms)) && vibrated) {
            return true;
        }
        _native = false;
        SDL_Log("%s", "rumble: physical emulation usage unavailable, using SDL haptic");
    }
#endif
    if (!_haptic) {
        return false;
    }
    if (!SDL_PlayHapticRumble(_haptic.get(), strength, pulse_ms)) {
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "rumble failed: %s", SDL_GetError());
        return false;
    }
    return true;
}

void Rumble::stop() {
#ifdef __ANDROID__
    if (_native) {
        bool unused = false;
        call_activity("rumbleStop", "()V", &unused, 0);
        return;
    }
#endif
    if (_haptic) {
        SDL_StopHapticRumble(_haptic.get());
    }
}

Rumble::Rumble() {
    if (!SDL_WasInit(SDL_INIT_HAPTIC)) {
        return;
    }
    int count = 0;
    SDL_HapticID* ids = SDL_GetHaptics(&count);
    if (!ids) {
        return;
    }
    for (int i = 0; i < count && !_haptic; ++i) {
        Haptic candidate(SDL_OpenHaptic(ids[i]));
        if (candidate && SDL_InitHapticRumble(candidate.get())) {
            _haptic = std::move(candidate);
            SDL_Log("rumble: using haptic device \"%s\"", SDL_GetHapticNameForID(ids[i]));
        }
    }
    SDL_free(ids);
    if (!_haptic) {
        SDL_Log("%s", "rumble: no usable haptic device, cartridge rumble is ignored");
    }
}

void Rumble::update(bool motor_on, u64 now_ns) {
    if (motor_on) {
        if (!_on || now_ns - _last_pulse_ns >= renew_ns) {
            pulse();
            _last_pulse_ns = now_ns;
        }
    } else if (_on) {
        stop();
    }
    _on = motor_on;
}

} // namespace bdmg
