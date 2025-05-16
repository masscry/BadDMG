#include "pixel-processor.hh"
#include "bus.hh"
#include "interrupt.hh"

#include <format>

namespace bdmg {

PixelProcessor::PixelProcessor(OnlySharedPtr) {
    std::ranges::fill(_memory, 0x00);
    std::ranges::fill(_framebuffer, 0x00);

}

SPPixelProcessor PixelProcessor::create() {
    return std::make_shared<PixelProcessor>(OnlySharedPtr{});
}

u8 PixelProcessor::read_byte(u16 addr) const {
    if (video_ram_range.has(addr)) {
        return _memory[addr - video_ram_range.start];
    }
    if (lcd_range.has(addr)) {
        switch (addr) {
        case reg_lcdc: return _regs.lcdc;
        case reg_stat: return _regs.stat;
        case reg_scy:  return _regs.scy;
        case reg_scx:  return _regs.scx;
        case reg_ly:   return _regs.ly;
        case reg_lyc:  return _regs.lyc;
        case reg_dma:  return _regs.dma;
        case reg_bgp:  return _regs.bgp;
        case reg_obp0: return _regs.obp0;
        case reg_obp1: return _regs.obp1;
        case reg_wy:   return _regs.wy;
        case reg_wx:   return _regs.wx;
        default: break;
        }
    }
    if (oam_range.has(addr)) {
        if (_oam_locked || _dma_active) {
            return 0xFF;
        }
        return _oam[addr - oam_range.start];
    }
    return 0xFF;
}

void PixelProcessor::write_byte(u16 addr, u8 data) {
    if (video_ram_range.has(addr)) {
        auto index = addr - video_ram_range.start;
        _memory[index] = data;
        return;
    }
    if (lcd_range.has(addr)) {
        switch (addr) {
        case reg_lcdc:
            _regs.lcdc = data; return;
        case reg_stat:
            _regs.stat = data; return;
        case reg_scy: 
            _regs.scy = data; return;
        case reg_scx:
            _regs.scx = data; return;
        case reg_ly:
            return;
        case reg_lyc:
            _regs.lyc = data; return;
        case reg_dma:
            _regs.dma = data;
            _dma_active = true;
            _dma_src = static_cast<u16>(data) << 8;
            _dma_count = 0;
            return;
        case reg_bgp: 
            _regs.bgp = data; return;
        case reg_obp0:
            _regs.obp0 = data; return;
        case reg_obp1:
            _regs.obp1 = data; return;
        case reg_wy:
            _regs.wy = data; return;
        case reg_wx:
            _regs.wx = data; return;
        default: break;
        }
    }
    if (oam_range.has(addr)) {
        if (_oam_locked || _dma_active) {
            return;
        }
        _oam[addr - oam_range.start] = data;
        return;
    }
}

void PixelProcessor::evaluate_sprites_for_line(u8 ly) {
    _visible_sprite_count = 0;
    const bool use_8x16 = (_regs.lcdc & obj_size);
    const u8 height = use_8x16 ? 16 : 8;

    for (size_t i = 0; i < 40 && _visible_sprite_count < 10; ++i) {
        const u8* entry = &_oam[i * 4];
        const u8 y_pos = entry[0];
        if (y_pos == 0 || y_pos >= 160) {
            continue;
        }
        
        const u8 effective_y = y_pos - 16;
        if (ly >= effective_y && ly < (effective_y + height)) {
            Sprite& sprite = _visible_sprites[_visible_sprite_count];
            sprite.y = y_pos;
            sprite.x = entry[1];
            sprite.tile = entry[2];
            sprite.flags = entry[3];
            sprite.oam_index = i;
            
            u8 line_offset = ly - effective_y;
            if (sprite.flags & 0x40) {
                line_offset = (height - 1) - line_offset;
            }
            
            if (use_8x16) {
                sprite.tile &= 0xFE;
                if (line_offset >= 8) {
                    sprite.tile++;
                    line_offset -= 8;
                }
            }
            
            sprite.tile_addr = 0x8000 + (sprite.tile * 16) + (line_offset * 2);
            sprite.tile_data_low = _memory[sprite.tile_addr - video_ram_range.start];
            sprite.tile_data_high = _memory[sprite.tile_addr + 1 - video_ram_range.start];
            
            _visible_sprite_count++;
        }
    }

    std::sort(_visible_sprites.begin(), _visible_sprites.begin() + _visible_sprite_count,
        [](const Sprite& a, const Sprite& b) {
            if (a.x != b.x) return a.x < b.x;
            return a.oam_index < b.oam_index;
        });
}

void PixelProcessor::render_scanline(u8 ly) {
    if (!(_regs.lcdc & bg_win_enable)) {
        std::fill_n(_framebuffer.begin() + ly * 160, 160, 0);
        return;
    }

    render_background_scanline(ly);

    if ((_regs.lcdc & window_enable) && ly >= _regs.wy && _regs.wx <= 166) {
        if (_window_line < 0) {
            _window_line = 0;
        }

        const u16 win_map_base = (_regs.lcdc & window_map_select) ? 0x9C00 : 0x9800;
        const int window_x = _regs.wx - 7;
        const bool signed_indexing = !(_regs.lcdc & bg_window_tile_select);
        const u16 tile_data_base = (_regs.lcdc & bg_window_tile_select) ? 0x8000 : 0x8800;

        if (_window_line < 144) {
            for (int x = std::max(0, window_x); x < 160; x++) {
                const u8 win_x = x - window_x;
                const u8 tile_y = (_window_line / 8) & 31;
                const u8 tile_x = (win_x / 8) & 31;
                const u8 fine_y = _window_line & 7;
                const u8 fine_x = win_x & 7;

                const u16 map_offset = (tile_y * 32) + tile_x;
                const u8 tile_num = _memory[win_map_base - video_ram_range.start + map_offset];

                u16 tile_addr = calculate_tile_address(tile_data_base, tile_num, signed_indexing);
                const u8 color = get_background_color(tile_addr, fine_x, fine_y);
                const u8 final_color = ((_regs.bgp >> (color * 2)) & 0x03);

                _framebuffer[ly * 160 + x] = final_color;
            }
            _window_line++;
        }
    }

    process_sprites(ly);
}

void PixelProcessor::render_background_scanline(u8 ly) {
    const u16 bg_map_base = (_regs.lcdc & bg_map_select) ? 0x9C00 : 0x9800;
    const u16 tile_data_base = (_regs.lcdc & bg_window_tile_select) ? 0x8000 : 0x8800;
    const bool signed_indexing = !(_regs.lcdc & bg_window_tile_select);

    const u8 y = (ly + _regs.scy) & 0xFF;
    const u8 tile_y = y >> 3;
    const u8 fine_y = y & 7;

    for (int x = 0; x < 160; x++) {
        const u8 mapped_x = (x + _regs.scx) & 0xFF;
        const u8 tile_x = mapped_x >> 3;
        const u8 fine_x = mapped_x & 7;

        const u16 map_addr = bg_map_base - video_ram_range.start + ((tile_y & 31) << 5) + (tile_x & 31);
        const u8 tile_num = _memory[map_addr];

        const u16 tile_addr = calculate_tile_address(tile_data_base, tile_num, signed_indexing);
        const u8 color = get_background_color(tile_addr, fine_x, fine_y);
        const u8 final_color = (_regs.bgp >> (color * 2)) & 0x03;

        _framebuffer[ly * 160 + x] = final_color;
    }
}

u16 PixelProcessor::calculate_tile_address(u16 tile_data_base, u8 tile_num, bool signed_indexing) const {
    if (signed_indexing) {
        return 0x9000 + static_cast<s16>(static_cast<s8>(tile_num)) * 16;
    }
    return tile_data_base + tile_num * 16;
}

u8 PixelProcessor::get_background_color(u16 tile_addr, u8 fine_x, u8 fine_y) const {
    const u16 adjusted_addr = tile_addr - video_ram_range.start + (fine_y * 2);
    const u8 tile_data_low = _memory[adjusted_addr];
    const u8 tile_data_high = _memory[adjusted_addr + 1];
    
    const u8 color_bit = 7 - fine_x;
    return ((tile_data_low >> color_bit) & 1) | 
           (((tile_data_high >> color_bit) & 1) << 1);
}

void PixelProcessor::render_frame() {
    if (_frame_ready_callback) {
        _frame_ready_callback(_framebuffer);
    }
}

void PixelProcessor::process_sprites(u8 ly) {
    if (!(_regs.lcdc & obj_enable)) {
        return;
    }

    for (int i = _visible_sprite_count - 1; i >= 0; i--) {
        const Sprite& sprite = _visible_sprites[i];
        
        const int screen_x_start = sprite.x - 8;
        
        for (int x = 0; x < 8; x++) {
            const int screen_x = screen_x_start + x;
            if (screen_x < 0 || screen_x >= 160) continue;

            const int bit = (sprite.flags & 0x20) ? x : (7 - x);
            const u8 color = ((sprite.tile_data_low >> bit) & 1) | 
                           (((sprite.tile_data_high >> bit) & 1) << 1);

            if (color == 0) continue;

            const int pixel_index = ly * 160 + screen_x;
            const u8 bg_pixel = _framebuffer[pixel_index];
            
            if (!(sprite.flags & 0x80) || bg_pixel == 0) {
                const u8 palette = (sprite.flags & 0x10) ? _regs.obp1 : _regs.obp0;
                _framebuffer[pixel_index] = (palette >> (color * 2)) & 0x03;
            }
        }
    }
}

void PixelProcessor::advance_dma(Bus& bus, u8 cycles) {
    if (!_dma_active) {
        return;
    }
    while (cycles > 0) {
        _oam[_dma_count] = bus.read_byte(_dma_src + _dma_count);
        _dma_count++;
        if (_dma_count >= dma_length) {
            _dma_active = false;
        }
        cycles -= 4;
    }
}

void PixelProcessor::advance(Bus& bus, u8 cycles) {
    advance_dma(bus, cycles);

    constexpr u8 mode_bits_mask = 0b11;
    constexpr u8 coincidence_flag = (1 << 2);
    constexpr u8 hblank_int_enable = (1 << 3);
    constexpr u8 vblank_int_enable = (1 << 4);
    constexpr u8 oam_int_enable = (1 << 5);
    constexpr u8 lyc_int_enable = (1 << 6);

    constexpr int oam_scan_cycles = 80;
    constexpr int max_pixel_transfer_cycles = 172;
    constexpr int min_hblank_cycles = 204;
    constexpr int line_cycles = oam_scan_cycles + max_pixel_transfer_cycles + min_hblank_cycles;

    if ((_regs.lcdc & lcd_enable) == 0) {
        _frame_cycles = 0;
        _regs.ly = 0;
        _mode = Mode::h_blank;
        _regs.stat = (_regs.stat & ~mode_bits_mask) | static_cast<u8>(_mode);
        _vblank_triggered = false;
        _last_mode = _mode;
        _oam_locked = false;
        _window_line = -1;
        return;
    }

    _frame_cycles += cycles;

    switch (_mode) {
        case Mode::oam_scan:
            if (_frame_cycles >= oam_scan_cycles) {
                _mode = Mode::draw;
                evaluate_sprites_for_line(_regs.ly);
            }
            break;
        case Mode::draw:
            if (_frame_cycles >= oam_scan_cycles + max_pixel_transfer_cycles) {
                _mode = Mode::h_blank;
                render_scanline(_regs.ly);
                if (_regs.stat & hblank_int_enable) {
                    notify_stat_interrupt(bus);
                }
            }
            break;
        case Mode::h_blank:
            if (_frame_cycles >= line_cycles) {
                _frame_cycles -= line_cycles;
                _regs.ly++;

                if (_regs.ly == 144) {
                    _mode = Mode::v_blank;
                    notify_vblank_interrupt(bus);
                    if (_regs.stat & vblank_int_enable) {
                        notify_stat_interrupt(bus);
                    }
                } else {
                    _mode = Mode::oam_scan;
                    if (_regs.stat & oam_int_enable) {
                        notify_stat_interrupt(bus);
                    }
                }
            }
            break;
        case Mode::v_blank:
            if (_frame_cycles >= line_cycles) {
                _frame_cycles -= line_cycles;
                _regs.ly++;
                
                if (_regs.ly >= 154) {
                    _regs.ly = 0;
                    _mode = Mode::oam_scan;
                    _vblank_triggered = false;
                    _window_line = -1;
                    if (_regs.stat & oam_int_enable) {
                        notify_stat_interrupt(bus);
                    }
                }
            }
            break;
    }

    if (_regs.ly == _regs.lyc) {
        _regs.stat |= coincidence_flag;
        if (_regs.stat & lyc_int_enable) {
            notify_stat_interrupt(bus);
        }
    } else {
        _regs.stat &= ~coincidence_flag;
    }

    _last_mode = _mode;
    _oam_locked = (_mode == Mode::oam_scan || _mode == Mode::draw);
    _regs.stat = (_regs.stat & ~mode_bits_mask) | static_cast<u8>(_mode);
}


void PixelProcessor::set_frame_ready_callback(FrameCallback callback) {
    _frame_ready_callback = std::move(callback);
}

void PixelProcessor::notify_vblank_interrupt(Bus& bus) {
    if (!_vblank_triggered) {
        _vblank_triggered = true;
        bus.write_byte(int_flag_addr, bus.read_byte(int_flag_addr) | int_vblank_bit);
        render_frame();
    }
}

void PixelProcessor::notify_stat_interrupt(Bus& bus) {
    bus.write_byte(int_flag_addr, bus.read_byte(int_flag_addr) | int_stat_bit);
}

} // namespace bdmg
