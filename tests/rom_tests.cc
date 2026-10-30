#include <gtest/gtest.h>

#include <bdmg/audio.hh>
#include <bdmg/bus.hh>
#include <bdmg/cartridge.hh>
#include <bdmg/cpu.hh>
#include <bdmg/high-ram.hh>
#include <bdmg/interrupt.hh>
#include <bdmg/joypad.hh>
#include <bdmg/pixel-processor.hh>
#include <bdmg/serial.hh>
#include <bdmg/timer.hh>
#include <bdmg/work-ram.hh>

#include "roms_embedded.hh"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <ios>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bdmg {

constexpr u16 final_result_addr = 0xA000;
constexpr u16 text_out_addr = 0xA004;
constexpr u8 result_running = 0x80;

constexpr u64 t_cycles_per_frame = 70224;
constexpr u32 default_timeout_minutes = 10;

u32 timeout_frames() {
    auto minutes = default_timeout_minutes;
    if (const auto* env = std::getenv("BDMG_ROM_TIMEOUT_MINUTES")) {
        minutes = static_cast<u32>(std::strtoul(env, nullptr, 10));
        if (minutes == 0) {
            minutes = default_timeout_minutes;
        }
    }
    return minutes * 60 * 60;
}

class CaptureSerial final : public Serial {
public:
    std::string text;

    void write_byte(u16 addr, u8 data) override {
        if (addr == 0xFF01 && (data == '\n' || data == '\r' ||
                               (data >= 0x20 && data <= 0x7E))) {
            text.push_back(static_cast<char>(data));
        }
        Serial::write_byte(addr, data);
    }
};

struct RomMachine {
    SPBus bus;
    SPPixelProcessor pp;
    SPAudio audio;
    SPSerial serial;
    Cpu cpu;

    std::shared_ptr<std::vector<u8>> last_frame;

    static RomMachine create(std::span<const u8> rom) {
        RomMachine machine;
        machine.last_frame = std::make_shared<std::vector<u8>>();

        auto cartridge = Cartridge::create_from_memory(rom);
        machine.bus = Bus::create();
        machine.bus->register_device(rom_bank_zero, cartridge);
        machine.bus->register_device(rom_bank_n, cartridge);
        machine.bus->register_device(ram_bank, cartridge);

        machine.bus->register_device(work_ram_range, WorkRam::create());

        machine.pp = PixelProcessor::create();
        machine.bus->register_device(video_ram_range, machine.pp);
        machine.bus->register_device(lcd_range, machine.pp);
        machine.bus->register_device(oam_range, machine.pp);

        machine.audio = Audio::create();
        machine.bus->register_device(audio_chan1, machine.audio);
        machine.bus->register_device(audio_chan2, machine.audio);
        machine.bus->register_device(audio_chan3, machine.audio);
        machine.bus->register_device(audio_chan4, machine.audio);
        machine.bus->register_device(audio_control, machine.audio);
        machine.bus->register_device(audio_waves, machine.audio);

        machine.bus->register_device(high_ram_range, HighRam::create());

        machine.bus->register_device(joypad_range, Joypad::create());
        machine.serial = std::make_shared<CaptureSerial>();
        machine.bus->register_device(serial_range, machine.serial);

        auto timer = TimerImpl::create();
        timer->set_div_reset(bdmg::make_div_reset_handler([audio = machine.audio]() { audio->reset_sequencer(); }));
        machine.bus->register_device(timer_range, timer);

        auto interrupt = Interrupt::create();
        machine.bus->register_device({ int_flag_addr, 1 }, interrupt);
        machine.bus->register_device({ int_enable_addr, 1 }, interrupt);

        machine.pp->set_frame_ready_callback(
            [frame = machine.last_frame](PixelProcessor::FrameSpan buf) {
                frame->assign(buf.begin(), buf.end());
            });

        machine.cpu = Cpu(machine.bus, nullptr);
        timer->advance(*machine.bus, 0x8);
        return machine;
    }
};

struct RomResult {
    u8 result_code = result_running;
    bool timed_out = false;
    bool never_started = false;
    bool from_serial = false;
    std::string text;
    std::string pc_trace;
};

std::string serial_text(const RomMachine& machine) {
    return std::static_pointer_cast<CaptureSerial>(machine.serial)->text;
}

std::optional<u8> serial_verdict(std::string_view text) {
    if (auto pos = text.find("Failed"); pos != std::string_view::npos) {
        u8 code = 1;
        if (auto hash = text.find('#', pos);
            hash != std::string_view::npos && hash + 1 < text.size()) {
            const char* begin = text.data() + hash + 1;
            char* end = nullptr;
            const auto parsed = std::strtoul(begin, &end, 10);
            if (end != begin && parsed >= 1 && parsed <= 254) {
                code = static_cast<u8>(parsed);
            }
        }
        return code;
    }
    if (text.find("Passed") != std::string_view::npos) {
        return 0;
    }
    return std::nullopt;
}

std::string read_text(const SPBus& bus) {
    std::string text;
    for (u16 addr = text_out_addr; text.size() < 256; ++addr) {
        auto ch = bus->read_byte(addr);
        if (ch == 0) {
            break;
        }
        if (ch < 0x20 || ch > 0x7E) {
            break;
        }
        text.push_back(static_cast<char>(ch));
    }
    return text;
}

constexpr u32 never_started_bail_frames = 600;

RomResult run(RomMachine& machine, bool has_protocol) {
    const auto frames = timeout_frames();
    u64 cycles = 0;
    bool started = false;
    std::vector<std::pair<u16, u32>> instr_trace;
    for (u32 frame = 0; frame < frames; ++frame) {
        // The trace only ever reports a ROM that never wrote the sentinel, so
        // stop recording once that outcome is off the table: it costs ~8 MB
        // per emulated second otherwise.
        const bool tracing = has_protocol && !started;
        while (cycles < t_cycles_per_frame) {
            cycles += machine.cpu.step();
            if (!tracing) {
                continue;
            }
            const auto pc = machine.cpu.regs().pc;
            if (instr_trace.empty() || instr_trace.back().first != pc) {
                instr_trace.emplace_back(pc, 1);
            } else {
                ++instr_trace.back().second;
            }
        }
        cycles -= t_cycles_per_frame;
        auto result = machine.bus->read_byte(final_result_addr);
        if (!started) {
            if (result == result_running) {
                started = true;
                continue;
            }
            if (!has_protocol) {
                if (auto verdict = serial_verdict(serial_text(machine))) {
                    return { *verdict, false, false, true,
                             serial_text(machine), {} };
                }
                continue;
            }
            if (frame >= never_started_bail_frames) {
                std::string trace;
                for (auto [pc, count]: instr_trace) {
                    trace += std::format("${:04X}", pc);
                    if (count > 1) {
                        trace += std::format("×{}", count);
                    }
                    trace += " ";
                }
                if (!trace.empty()) {
                    trace.pop_back();
                }
                return { result_running, false, true, false,
                         serial_text(machine), trace };
            }
            continue;
        }
        if (result != result_running) {
            return { result, false, false, false, read_text(machine.bus), {} };
        }
    }
    return { result_running, true, false, false, serial_text(machine), {} };
}

void dump_frame(std::string_view stem, std::span<const u8> frame) {
    if (frame.empty()) {
        return;
    }
    constexpr u8 mapping[4] = { 0xFF, 0xAA, 0x55, 0x00 };
    auto out = std::ofstream(
        std::format("failed-{}.pgm", stem), std::ios::binary);
    out << std::format("P5\n{} {}\n255\n", screen_size.x, screen_size.y);
    for (auto src: frame) {
        out.write(reinterpret_cast<const char*>(&mapping[src & 0x03]), 1);
    }
}

std::string sanitize_name(std::string_view name) {
    std::string out(name);
    for (auto& ch: out) {
        if (!std::isalnum(static_cast<unsigned char>(ch))) {
            ch = '_';
        }
    }
    return out;
}

class RomTests : public testing::Test {};

// Raw text mirror at $A004 (see the gb-test-roms readme.txt: text
// output is appended to a zero-terminated string there).
std::string text_mirror_hex(const RomMachine& machine) {
    std::string out;
    for (u16 addr = 0xA004; addr < 0xA080; ++addr) {
        out += std::format("${:02X} ", machine.bus->read_byte(addr));
    }
    return out;
}

// Registers worth dumping when a ROM hangs before writing the sentinel.
std::string debug_state(const RomMachine& machine) {
    auto hex = [&](u16 addr) {
        return std::format("${:02X}", machine.bus->read_byte(addr));
    };
    const u16 pc = machine.cpu.regs().pc;
    std::string code;
    for (int offset = -8; offset < 40; ++offset) {
        const auto addr = static_cast<u16>(pc + offset);
        code += std::format("${:02X} ", machine.bus->read_byte(addr));
    }
    return std::format(
        "A000={} LY={} DIV={} TIMA={} IF={} IE={} SC={} "
        "PC={} code[{}] A004[{}]",
        hex(0xA000), hex(0xFF44), hex(0xFF04), hex(0xFF05),
        hex(0xFF0F), hex(0xFFFF), hex(0xFF02),
        std::format("${:04X}", pc), code, text_mirror_hex(machine));
}

void expect_pass(const embedded_roms::Rom& rom) {
    constexpr u8 sentinel_write[] = {0x3E, 0x80, 0xEA, 0x00, 0xA0};
    const bool has_protocol = std::search(
        rom.data, rom.end,
        std::begin(sentinel_write), std::end(sentinel_write)) != rom.end;

    auto machine = RomMachine::create(std::span<const u8>{rom.data, rom.end});
    auto result = run(machine, has_protocol);

    const auto stem = sanitize_name(rom.name);
    if (result.never_started) {
        dump_frame(stem, *machine.last_frame);
        FAIL() << rom.name << ": $A000 never became the $80 sentinel within "
               << never_started_bail_frames
               << " frames — either the ROM doesn't use the Blargg "
               << "cartridge-RAM result protocol (old cpu_instrs/instr_timing "
               << "shells) or its shell hung before writing it; outcome "
               << "unknown. " << debug_state(machine)
               << "; PC trace: " << result.pc_trace
               << (result.text.empty() ? "" : "; serial output: " + result.text);
    }
    if (result.timed_out) {
        dump_frame(stem, *machine.last_frame);
        FAIL() << rom.name << ": timed out after " << timeout_frames()
               << " frames (set BDMG_ROM_TIMEOUT_MINUTES to adjust)"
               << (result.text.empty() ? "" : "; serial output: " + result.text);
    }
    if (result.result_code != 0) {
        dump_frame(stem, *machine.last_frame);
        FAIL() << rom.name << ": result code "
               << static_cast<int>(result.result_code)
               << " (" << (result.result_code == 255 ? "internal error" : "failed")
               << "), " << (result.from_serial ? "serial" : "ROM")
               << " output: " << result.text
               << "; A004: " << text_mirror_hex(machine);
    }
}

TEST_F(RomTests, cpu_instrs) { expect_pass(embedded_roms::cpu_instrs); }
TEST_F(RomTests, dmg_sound) { expect_pass(embedded_roms::dmg_sound); }
TEST_F(RomTests, halt_bug) { expect_pass(embedded_roms::halt_bug); }
TEST_F(RomTests, instr_timing) { expect_pass(embedded_roms::instr_timing); }
TEST_F(RomTests, mem_timing) { expect_pass(embedded_roms::mem_timing); }
TEST_F(RomTests, DISABLED_interrupt_time) { expect_pass(embedded_roms::interrupt_time); }
TEST_F(RomTests, DISABLED_oam_bug) { expect_pass(embedded_roms::oam_bug); }

namespace {

fs::path make_test_rom(const fs::path& dir, Cartridge::Type type, u8 ram_code) {
    auto path = dir / "game.gb";
    std::vector<u8> rom(0x8000, 0xFF);
    rom[0x147] = static_cast<u8>(type);
    rom[0x149] = ram_code;
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(rom.data()),
              static_cast<std::streamsize>(rom.size()));
    return path;
}

} // namespace

TEST(CartridgeSave, battery_ram_is_live_mapped) {
    const auto dir = fs::temp_directory_path() / "baddmg-cartridge-save-test";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const auto rom_path = make_test_rom(dir, Cartridge::Type::mbc1_ram_bat, 0x02);
    const auto sav_path = dir / "game.sav";

    const std::vector<u8> initial { 0x01, 0x02, 0x03 };
    {
        std::ofstream out(sav_path, std::ios::binary);
        out.write(reinterpret_cast<const char*>(initial.data()),
                  static_cast<std::streamsize>(initial.size()));
    }

    {
        auto cartridge = Cartridge::create(rom_path);
        cartridge->write_byte(0x0000, 0x0A);
        EXPECT_EQ(cartridge->read_byte(0xA000), 0x01);
        EXPECT_EQ(cartridge->read_byte(0xA001), 0x02);
        EXPECT_EQ(cartridge->read_byte(0xA002), 0x03);
        EXPECT_EQ(cartridge->read_byte(0xA003), 0x00);

        cartridge->write_byte(0xA000, 0x42);

        // The file is the live SRAM: the write is visible without exit.
        std::ifstream live(sav_path, std::ios::binary);
        char first = 0;
        live.read(&first, 1);
        EXPECT_EQ(static_cast<u8>(first), 0x42);
    }

    EXPECT_EQ(fs::file_size(sav_path), 8192ULL);
    {
        std::ifstream in(sav_path, std::ios::binary);
        std::vector<u8> data(8192);
        in.read(reinterpret_cast<char*>(data.data()),
                static_cast<std::streamsize>(data.size()));
        EXPECT_EQ(data[0], 0x42);
        EXPECT_EQ(data[1], 0x02);
        EXPECT_EQ(data[2], 0x03);
        EXPECT_EQ(data[3], 0x00);
    }

    {
        auto cartridge = Cartridge::create(rom_path);
        cartridge->write_byte(0x0000, 0x0A);
        EXPECT_EQ(cartridge->read_byte(0xA000), 0x42);
        EXPECT_EQ(cartridge->read_byte(0xA001), 0x02);
        EXPECT_EQ(cartridge->read_byte(0xA002), 0x03);
    }

    fs::remove_all(dir);
}

TEST(CartridgeRtc, persists_and_catches_up_while_closed) {
    const auto dir = fs::temp_directory_path() / "baddmg-cartridge-rtc-test";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const auto rom_path = make_test_rom(dir, Cartridge::Type::mbc3_timer_ram_bat, 0x02);
    const auto rtc_path = dir / "game.rtc";

    auto now = std::chrono::system_clock::from_time_t(1'700'000'000);
    MapperClock clock = [&now] { return now; };

    {
        auto cartridge = Cartridge::create(rom_path, clock);
        cartridge->write_byte(0x0000, 0x0A);
        cartridge->write_byte(0x4000, 0x0A);  // RTC hours
        cartridge->write_byte(0xA000, 12);
        cartridge->write_byte(0x4000, 0x09);  // RTC minutes
        cartridge->write_byte(0xA000, 34);
        cartridge->write_byte(0x4000, 0x08);  // RTC seconds
        cartridge->write_byte(0xA000, 56);

        cartridge->write_byte(0x6000, 0x00);
        cartridge->write_byte(0x6000, 0x01);  // latch
        cartridge->write_byte(0x4000, 0x0A);
        EXPECT_EQ(cartridge->read_byte(0xA000), 12);
        cartridge->write_byte(0x4000, 0x09);
        EXPECT_EQ(cartridge->read_byte(0xA000), 34);
        cartridge->write_byte(0x4000, 0x08);
        EXPECT_EQ(cartridge->read_byte(0xA000), 56);
    }
    EXPECT_EQ(fs::file_size(rtc_path), 24ULL);

    now += std::chrono::hours(1);

    {
        auto cartridge = Cartridge::create(rom_path, clock);
        cartridge->write_byte(0x0000, 0x0A);
        cartridge->write_byte(0x4000, 0x0A);
        cartridge->write_byte(0x6000, 0x00);
        cartridge->write_byte(0x6000, 0x01);  // latch
        EXPECT_EQ(cartridge->read_byte(0xA000), 13);
        cartridge->write_byte(0x4000, 0x09);
        EXPECT_EQ(cartridge->read_byte(0xA000), 34);
        cartridge->write_byte(0x4000, 0x08);
        EXPECT_EQ(cartridge->read_byte(0xA000), 56);
    }

    fs::remove_all(dir);
}

TEST(PpuTiming, lyc_zero_match_fires_before_visible_line_zero) {
    auto bus = Bus::create();
    auto pp = PixelProcessor::create();
    bus->register_device(lcd_range, pp);
    auto interrupt = Interrupt::create();
    bus->register_device({ int_flag_addr, 1 }, interrupt);
    bus->register_device({ int_enable_addr, 1 }, interrupt);

    bus->write_byte(reg_stat, 0x40);
    bus->write_byte(reg_lyc, 0x00);
    bus->write_byte(reg_lcdc, 0x91);

    constexpr int max_advances = 2 * 70224 / 4;
    int advances = 0;
    while (bus->read_byte(reg_ly) != 152 && advances < max_advances) {
        bus->advance();
        ++advances;
    }
    ASSERT_LT(advances, max_advances);

    bus->write_byte(int_flag_addr, 0x00);

    while (!(bus->read_byte(int_flag_addr) & int_stat_bit) && advances < max_advances) {
        bus->advance();
        ++advances;
    }
    ASSERT_LT(advances, max_advances);
    EXPECT_EQ(bus->read_byte(reg_ly), 0);
    EXPECT_EQ(bus->read_byte(reg_stat) & 0x03, 1);
}

} // namespace bdmg
