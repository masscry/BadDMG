#pragma once

#include <cstdint>
#include <cstdlib>

#include <string_view>
#include <source_location>

namespace bdmg {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using s8 = std::int8_t;
using s16 = std::int16_t;
using s32 = std::int32_t;
using s64 = std::int64_t;

using f32 = float;
using f64 = double;

template<typename Data>
struct Vec2 {
    Data x;
    Data y;
};

template<typename Data>
struct Range {
    Data start;
    Data length;

    friend constexpr 
    bool operator < (const Range<Data>& lhs, const Range<Data>& rhs) {
        return (lhs.start < rhs.start) && ((lhs.start+lhs.length) < (rhs.start+rhs.length));
    }

    constexpr
    bool has(Data pos) const {
        return (start <= pos) && (pos < (start+length));
    }

    constexpr
    bool overlap(const Range<Data>& other) const {
        return has(other.start) || has(other.start+other.length);
    }

};

using Vec2f = Vec2<float>;
using Vec2s = Vec2<s32>;
using RangeU16 = Range<u16>;
using RangeU32 = Range<u32>;

struct Col4 {
    u8 r;
    u8 g;
    u8 b;
    u8 a;
};

void add_last_words(std::string_view message);

[[noreturn]] void panic(std::string_view message, const std::source_location location = std::source_location::current()) noexcept;

template<typename Condition>
void expected(Condition&& condition, std::string_view message, const std::source_location& location = std::source_location::current()) {
    if (!std::forward<Condition>(condition)) {
        panic(message, location);
    }
}

template<typename Func>
requires requires (Func func) { noexcept(func()); }
auto defer(Func&& func) {
    class Deferred {
    public:
        Deferred(Func&& func) : _func(std::forward<Func>(func)) {}
        ~Deferred() { _func(); }
    private:
        Func _func;
    };
    return Deferred{ std::forward<Func>(func) };
};

} // namespace bdmg
