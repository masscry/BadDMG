#pragma once

#include "common.hh"
#include "device.hh"

#include <array>
#include <span>
#include <functional>

namespace bdmg {

class Bus;

constexpr Vec2s screen_size { .x = 160, .y = 144 };

using SPPixelProcessor = std::shared_ptr<class PixelProcessor>;

constexpr auto video_ram_range = RangeU16 { 0x8000, 0x2000 };
constexpr auto lcd_range = RangeU16 { 0xFF40, 12 };
constexpr auto oam_range = RangeU16 { 0xFE00, 0xA0 };
constexpr int max_sprites_per_line = 10;
constexpr u8 sprite_y_offset = 16;
constexpr u8 sprite_x_offset = 8;

enum LcdRegisterId : u16 {
    reg_lcdc = 0xFF40,
    reg_stat = 0xFF41,
    reg_scy  = 0xFF42,
    reg_scx  = 0xFF43,
    reg_ly   = 0xFF44,
    reg_lyc  = 0xFF45,
    reg_dma  = 0xFF46,
    reg_bgp  = 0xFF47,
    reg_obp0 = 0xFF48,
    reg_obp1 = 0xFF49,
    reg_wy   = 0xFF4A,
    reg_wx   = 0xFF4B
};

class PixelProcessor : public ITickingDevice {
protected:
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:
    explicit PixelProcessor(OnlySharedPtr);
    static SPPixelProcessor create();

    u8 read_byte(u16 addr) const override;
    void write_byte(u16, u8) override;
    void advance(Bus&, u8 cycles) final;

    std::size_t id() const final {
        return DeviceTypeId::ppu;
    }

    using FrameSpan = std::span<const u8, screen_size.x * screen_size.y>;
    using FrameCallback = std::function<void(FrameSpan)>;

    void set_frame_ready_callback(FrameCallback callback);

private:
    struct Sprite {
        u8 x;
        u8 y;
        u8 tile;
        u8 flags;
        u16 tile_addr;
        u8 tile_data_low;
        u8 tile_data_high;
        size_t oam_index;
    };

    union Registers {
        struct {
            u8 lcdc = 0x00;
            u8 stat = 0x00;
            u8 scy = 0x00;
            u8 scx = 0x00;
            u8 ly = 0x00;
            u8 lyc = 0x00;
            u8 dma = 0x00;
            u8 bgp = 0xFC;
            u8 obp0 = 0xFF;
            u8 obp1 = 0xFF;
            u8 wy = 0x00;
            u8 wx = 0x00;
        };
        u8 list[12];
    };

    enum class Mode {
        h_blank = 0,
        v_blank = 1,
        oam_scan = 2,
        draw = 3
    };

    enum LcdControlBits : u8 {
        bg_win_enable = 0x01,
        obj_enable = 0x02,
        obj_size = 0x04,
        bg_map_select = 0x08,
        bg_window_tile_select = 0x10,
        window_enable = 0x20,
        window_map_select = 0x40,
        lcd_enable = 0x80,
    };

    std::array<u8, video_ram_range.length> _memory;
    std::array<u8, screen_size.x*screen_size.y> _framebuffer;
    Mode _mode = Mode::oam_scan;
    Registers _regs {};
    u64 _frame_cycles = 0;

    int _window_line = -1;

    bool _vblank_triggered = false;
    Mode _last_mode = Mode::h_blank;

    std::array<u8, oam_range.length> _oam;
    bool _oam_locked = false;

    std::array<Sprite, max_sprites_per_line> _visible_sprites;
    u8 _visible_sprite_count = 0;

    FrameCallback _frame_ready_callback;

    bool _dma_active = false;
    u16 _dma_src = 0;
    u8 _dma_count = 0;

    static constexpr u8 dma_length = 0xA0;

    void advance_dma(Bus& bus, u8 cycles);

    void render_frame();

    void evaluate_sprites_for_line(u8);
    void render_scanline(u8);
    void process_sprites(u8);
    
    void notify_vblank_interrupt(Bus&);
    void notify_stat_interrupt(Bus&);

    void render_background_scanline(u8);
    u8 get_background_color(u16 tile_addr, u8 fine_x, u8 fine_y) const;
    u16 calculate_tile_address(u16 tile_data_base, u8 tile_num, bool signed_indexing) const;
};

} // namespace bdmg
