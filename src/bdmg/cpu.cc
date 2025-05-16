#include "cpu.hh"

#include "interrupt.hh"

#include <iostream>
#include <format>
#include <bit>

namespace bdmg {

enum Alu : u8 {
    zero = 0x80,
    sub = 0x40,
    halfcarry = 0x20,
    carry = 0x10
};

template<u8 opcode>
void Cpu::hcf() {
    std::cerr << "hcf" << ' ' << std::hex << static_cast<int>(opcode) << std::endl;
    std::cerr << print_state();
    panic("halt-and-catch-fire");
}

template<u16 addr>
void Cpu::rst() {
    _cycle_count += _bus->advance();
    write_byte(--_regs.sp, (_regs.pc) >> 8);
    write_byte(--_regs.sp, (_regs.pc) & 0xFF);
    _regs.pc = addr;
}

template<auto dst>
void Cpu::ld_dst_i8() {
    _regs.*dst = imm8();
}

template<auto dst, auto src>
void Cpu::ld_dst_src() {
    _regs.*dst = _regs.*src;
}

template<auto src>
void Cpu::ld_dhl_src() {
    write_byte(_regs.hl, std::invoke(src, _regs));
}

template<auto dst>
void Cpu::ld_dst_dhl() {
    _regs.*dst = read_byte(_regs.hl);
}

template<auto dst>
void Cpu::ld_dst_a() {
    write_byte(_regs.*dst, _regs.a);
}

template<auto dst>
void Cpu::inc_dst8() {
    _regs.*dst += 1;
    _regs.f &= ~(Alu::sub | Alu::zero | Alu::halfcarry);
    _regs.f |= (((_regs.*dst & 0x0F) == 0) * Alu::halfcarry);
    _regs.f |= ((_regs.*dst == 0) * Alu::zero);
}

template<auto dst>
void Cpu::dec_dst8() {
    _regs.*dst -= 1;
    _regs.f &= ~(Alu::zero | Alu::halfcarry);
    _regs.f |= Alu::sub;
    _regs.f |= (Alu::halfcarry * ((_regs.*dst & 0x0F) == 0x0F));
    _regs.f |= (Alu::zero * (_regs.*dst == 0));    
}

template<auto src>
void Cpu::add_hl_src() {
    u16 hl = _regs.hl;
    u16 rr;
    _cycle_count += _bus->advance();
    rr = _regs.*src;
    _regs.hl = hl + rr;
    _regs.f &= ~(Alu::sub | Alu::carry | Alu::halfcarry);

    if (((hl & 0xFFF) + (rr & 0xFFF)) & 0x1000) {
        _regs.f |= Alu::halfcarry;
    }

    if ( ((unsigned) hl + (unsigned) rr) & 0x10000) {
        _regs.f |= Alu::carry;
    }
}

template<auto src>
void Cpu::add_a_src() {
    u16 res = _regs.a + _regs.*src;
    _regs.f = (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) + (_regs.*src & 0x0F)) > 0x0F) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

template<auto src>
void Cpu::adc_a_src() {
    u16 carry = (_regs.f & Alu::carry) ? 1 : 0;
    u16 res = _regs.a + _regs.*src + carry;
    _regs.f = (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) + (_regs.*src & 0x0F) + carry) > 0x0F) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

template<auto src>
void Cpu::sub_a_src() {
    u16 res = _regs.a - _regs.*src;
    _regs.f = Alu::sub |
              (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) - (_regs.*src & 0x0F)) < 0) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

template<auto src>
void Cpu::sbc_a_src() {
    u16 carry = (_regs.f & Alu::carry) ? 1 : 0;
    u16 res = _regs.a - _regs.*src - carry;
    _regs.f = Alu::sub |
              (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) - (_regs.*src & 0x0F) - carry) < 0) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

template<auto src>
void Cpu::and_dst_src() {
    _regs.a &= _regs.*src;
    _regs.f = Alu::halfcarry | ((_regs.a == 0) * Alu::zero);
}

template<auto src>
void Cpu::xor_a_src() {
    _regs.a ^= _regs.*src;
    _regs.f = (_regs.a == 0) * Alu::zero;
}

template<auto src>
void Cpu::or_a_src() {
    _regs.a |= _regs.*src;
    _regs.f = (_regs.a == 0) * Alu::zero;
}

template<auto src>
void Cpu::cp_a_src() {
    u16 res = _regs.a - _regs.*src;
    _regs.f = Alu::sub |
              (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) - (_regs.*src & 0x0F)) < 0) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
}

u8 Cpu::imm8() {
    return read_byte(_regs.pc++);
}

u16 Cpu::imm16() {
    const auto result = read_word(_regs.pc);
    _regs.pc += 2;
    return result;
}

template<auto dst>
void Cpu::inc_dst16() {
    _cycle_count += _bus->advance();
    _regs.*dst += 1;
}

template<auto dst>
void Cpu::dec_dst16() {
    _cycle_count += _bus->advance();
    _regs.*dst -= 1;
}

template<auto arg>
void Cpu::cb_rlc(){
    u8 data = _regs.*arg;
    bool carry = (data & 0x80) != 0;
    _regs.f = 0x00;
    data = (data << 1) | carry;
    _regs.*arg = data;
    _regs.f |= (Alu::carry * carry);
    _regs.f |= (Alu::zero * (data == 0));
}

template<auto arg>
void Cpu::cb_rrc(){
    u8 data = _regs.*arg;
    bool carry = (data & 0x01) != 0;
    _regs.f = 0x00;
    data = (data >> 1) | (carry << 7);
    _regs.*arg = data;
    _regs.f |= (Alu::carry * carry);
    _regs.f |= (Alu::zero * (data == 0));
}

template<auto arg>
void Cpu::cb_rl(){
    u8 data = _regs.*arg;
    bool old_carry = (_regs.f & Alu::carry) != 0;
    bool new_carry = (data & 0x80) != 0;
    _regs.f = 0x00;
    data = (data << 1) | old_carry;
    _regs.*arg = data;
    _regs.f |= (Alu::carry * new_carry);
    _regs.f |= (Alu::zero * (data==0));
}

template<auto arg>
void Cpu::cb_rr(){
    u8 data = _regs.*arg;
    bool carry = (_regs.f & Alu::carry) != 0;
    bool bit1 = (data & 0x01) != 0;

    _regs.f = 0x00;
    data = (data >> 1) | (carry << 7);
    _regs.*arg = data;
    _regs.f |= (Alu::carry * bit1);
    _regs.f |= (Alu::zero * (data == 0));
}

template<auto arg>
void Cpu::cb_sla(){
    u8 data = _regs.*arg;
    bool carry = (data & 0x80) != 0;
    data = data << 1;
    _regs.*arg = data;
    _regs.f = 0x00;
    _regs.f |= (Alu::carry * carry);
    _regs.f |= (Alu::zero * (data == 0));
}

template<auto arg>
void Cpu::cb_sra(){
    u8 data = _regs.*arg;
    u8 bit7_mask = data & 0x80;
    _regs.f = 0x00;
    _regs.f |= ((Alu::carry) * ((data&1)!=0));
    data = (data >> 1) | bit7_mask;
    _regs.*arg = data;
    _regs.f |= (Alu::zero * (data == 0));
}

template<auto arg>
void Cpu::cb_swap(){
    u8 data = _regs.*arg;
    _regs.f = 0x00;
    _regs.*arg = (data >> 4) | (data << 4);
    _regs.f |= (Alu::zero * (!data));
}

template<auto arg>
void Cpu::cb_srl(){
    u8 data = _regs.*arg;
    bool carry = (data & 0x01) != 0;
    data = data >> 1;
    _regs.*arg = data;
    _regs.f = 0x00;
    _regs.f |= (Alu::carry * carry);
    _regs.f |= (Alu::zero * (data == 0));
}

template<auto arg, u8 bit>
void Cpu::cb_test(){
    u8 data = _regs.*arg;
    _regs.f &= Alu::carry;
    _regs.f |= Alu::halfcarry;
    _regs.f |= (Alu::zero * (!(bit & data))); 
}

template<auto arg, u8 bit>
void Cpu::cb_set(){
    _regs.*arg |= bit;
}

template<auto arg, u8 bit>
void Cpu::cb_reset(){
    _regs.*arg &= ~bit;
}

void Cpu::cb_rlc_m() {
    u8 data = read_byte(_regs.hl);
    bool carry = (data & 0x80) != 0;
    _regs.f = 0x00;
    data = (data << 1) | carry;
    write_byte(_regs.hl, data);
    _regs.f |= (Alu::carry * carry);
    _regs.f |= (Alu::zero * (data == 0));
}

void Cpu::cb_rrc_m() {
    u8 data = read_byte(_regs.hl);
    bool carry = (data & 0x01) != 0;
    _regs.f = 0x00;
    data = (data >> 1) | (carry << 7);
    write_byte(_regs.hl, data);
    _regs.f |= (Alu::carry * carry);
    _regs.f |= (Alu::zero * (data == 0));
}

void Cpu::cb_rl_m() {
    u8 data = read_byte(_regs.hl);
    bool old_carry = (_regs.f & Alu::carry) != 0;
    bool new_carry = (data & 0x80) != 0;
    _regs.f = 0x00;
    data = (data << 1) | old_carry;
    write_byte(_regs.hl, data);
    _regs.f |= (Alu::carry * new_carry);
    _regs.f |= (Alu::zero * (data == 0));
}

void Cpu::cb_rr_m() {
    u8 data = read_byte(_regs.hl);
    bool old_carry = (_regs.f & Alu::carry) != 0;
    bool new_carry = (data & 0x01) != 0;
    _regs.f = 0x00;
    data = (data >> 1) | (old_carry << 7);
    write_byte(_regs.hl, data);
    _regs.f |= (Alu::carry * new_carry);
    _regs.f |= (Alu::zero * (data == 0));
}

void Cpu::cb_sla_m() {
    u8 data = read_byte(_regs.hl);
    bool carry = (data & 0x80) != 0;
    data = data << 1;
    write_byte(_regs.hl, data);
    _regs.f = 0x00;
    _regs.f |= (Alu::carry * carry);
    _regs.f |= (Alu::zero * (data == 0));
}

void Cpu::cb_sra_m() {
    u8 data = read_byte(_regs.hl);
    u8 bit7_mask = data & 0x80;
    _regs.f = 0x00;
    _regs.f |= ((Alu::carry) * ((data&1)!=0));
    data = (data >> 1) | bit7_mask;
    write_byte(_regs.hl, data);
    _regs.f |= (Alu::zero * (data == 0));
}

void Cpu::cb_swap_m() {
    u8 data = read_byte(_regs.hl);
    _regs.f = 0x00;
    data = (data >> 4) | (data << 4);
    write_byte(_regs.hl, data);
    _regs.f |= (Alu::zero * (!data));
}

void Cpu::cb_srl_m() {
    u8 data = read_byte(_regs.hl);
    bool carry = (data & 0x01) != 0;
    data = data >> 1;
    write_byte(_regs.hl, data);
    _regs.f = 0x00;
    _regs.f |= (Alu::carry * carry);
    _regs.f |= (Alu::zero * (data == 0));
}

template<u8 bit>
void Cpu::cb_test_m() {
    u8 data = read_byte(_regs.hl);
    _regs.f &= Alu::carry;
    _regs.f |= Alu::halfcarry;
    _regs.f |= (Alu::zero * ((data & bit) == 0));
}

template<u8 bit>
void Cpu::cb_set_m() {
    u8 data = read_byte(_regs.hl);
    data |= bit;
    write_byte(_regs.hl, data);
}

template<u8 bit>
void Cpu::cb_reset_m() {
    u8 data = read_byte(_regs.hl);
    data &= ~bit;
    write_byte(_regs.hl, data);
}

Cpu::Cpu(const SPBus& bus, const SPBootRom& boot_rom)
    : _bus(bus)
    , _boot_rom(boot_rom)
    , _just_wrote_if(false)
{
    if (boot_rom) {
        _regs.af = 0x0000;
        _regs.bc = 0x0000;
        _regs.de = 0x0000;
        _regs.hl = 0x0000;
        _regs.sp = 0x0000;
        _regs.pc = 0x0000;
        _read_byte = &Cpu::read_boot_byte;
        _write_byte = &Cpu::write_boot_byte;
    } else {
        _regs.af = 0x01B0;
        _regs.bc = 0x0013;
        _regs.de = 0x00D8;
        _regs.hl = 0x014D;
        _regs.sp = 0xFFFE;
        _regs.pc = 0x0100;
        
        constexpr std::pair<u16, u8> lcd_init_values[] = {
            {reg_lcdc, 0x91},
            {reg_stat, 0x00},
            {reg_scy,  0x00},
            {reg_scx,  0x00},
            {reg_ly,   0x00},
            {reg_lyc,  0x00},
            {reg_bgp,  0xFC},
            {reg_obp0, 0xFF},
            {reg_obp1, 0xFF},
            {reg_wy,   0x00},
            {reg_wx,   0x00}
        };

        for (const auto& [reg, value] : lcd_init_values) {
            _bus->write_byte(reg, value);
        }

        _read_byte = &Cpu::read_bus_byte;
        _write_byte = &Cpu::write_bus_byte;
    }
}

std::string Cpu::print_state() {
    return std::format(
        "[{:04X}] AF:{:04X} BC:{:04X} DE:{:04X} HL:{:04X} SP:{:04X} PC:{:04X} | {:02X} {:02X} {:02X} {:02X}",
        _cycle_count,
        _regs.af, _regs.bc, _regs.de, _regs.hl, _regs.sp, _regs.pc,
        _bus->read_byte(_regs.pc), _bus->read_byte(_regs.pc + 1), _bus->read_byte(_regs.pc + 2), _bus->read_byte(_regs.pc + 3)
    );
}

void Cpu::nop() {};

inline
void Cpu::ld_hi_a() {
    u8 hi_addr = imm8();
    write_byte(0xFF00 + hi_addr, _regs.a);
}

inline
void Cpu::ld_a_hi() {
    u8 hi_addr = imm8();
    _regs.a = read_byte(0xFF00 + hi_addr);
}

template<auto src>
void Cpu::ld_a_drr() {
    _regs.a = read_byte(_regs.*src);
}

template<auto dst>
void Cpu::ldr_i16() {
    _regs.*dst = imm16();
}

void Cpu::ld_hldec() {
    write_byte(_regs.hl--, _regs.a);
}

void Cpu::ld_hlinc() {
    write_byte(_regs.hl++, _regs.a);
}

void Cpu::ld_a_hlinc() {
    _regs.a = read_byte(_regs.hl++);
}

void Cpu::ld_a_hldec() {
    _regs.a = read_byte(_regs.hl--);
}

void Cpu::ldhic_a() {
    write_byte(0xFF00 + _regs.c, _regs.a);
}

void Cpu::lda_hic() {
    _regs.a = read_byte(0xFF00 + _regs.c);
}

void Cpu::ld_di16_a() {
    write_byte(imm16(), _regs.a);
}

void Cpu::ld_a_di16() {
    _regs.a = read_byte(imm16());
}

void Cpu::jp() {
    auto new_pc = read_word(_regs.pc);
    _cycle_count += _bus->advance();
    _regs.pc = new_pc;
}

void Cpu::jp_hl() {
    _regs.pc = _regs.hl;
}

void Cpu::jp_nc() {
    auto new_pc = imm16();
    if ((_regs.f & carry) == 0) {
        _cycle_count += _bus->advance();
        _regs.pc = new_pc;
    }
}

void Cpu::jp_nz() {
    auto new_pc = imm16();
    if ((_regs.f & zero) == 0) {
        _cycle_count += _bus->advance();
        _regs.pc = new_pc;
    }
}

void Cpu::jp_c() {
    auto new_pc = imm16();
    if (_regs.f & carry) {
        _cycle_count += _bus->advance();
        _regs.pc = new_pc;
    }
}

void Cpu::jp_z() {
    auto new_pc = imm16();
    if (_regs.f & zero) {
        _cycle_count += _bus->advance();
        _regs.pc = new_pc;
    }
}

void Cpu::jr() {
    s8 offset = signed_read_byte(_regs.pc++);
    _cycle_count += _bus->advance();
    _regs.pc += offset;
}

template<auto cond>
void Cpu::jr_cc() {
    s8 offset = signed_read_byte(_regs.pc++);
    if (cond(_regs.f)) {
        _cycle_count += _bus->advance();
        _regs.pc += offset;
    }
}

void Cpu::call_r16() {
    u16 jump_addr = imm16();
    _cycle_count += _bus->advance();
    push_sp_word(_regs.pc);
    _regs.pc = jump_addr;
}

void Cpu::call_nz() {
    u16 jump_addr = imm16();
    if ((_regs.f & Alu::zero) == 0) {
        _cycle_count += _bus->advance();
        push_sp_word(_regs.pc);
        _regs.pc = jump_addr;
    }
}

void Cpu::call_nc() {
    u16 jump_addr = imm16();
    if ((_regs.f & Alu::carry) == 0) {
        _cycle_count += _bus->advance();
        push_sp_word(_regs.pc);
        _regs.pc = jump_addr;
    }
}

void Cpu::call_z() {
    u16 jump_addr = imm16();
    if (_regs.f & Alu::zero) {
        _cycle_count += _bus->advance();
        push_sp_word(_regs.pc);
        _regs.pc = jump_addr;
    }
}

void Cpu::call_c() {
    u16 jump_addr = imm16();
    if (_regs.f & Alu::carry) {
        _cycle_count += _bus->advance();
        push_sp_word(_regs.pc);
        _regs.pc = jump_addr;
    }
}

void Cpu::ret() {
    u16 ret_addr = read_word(_regs.sp);
    _regs.sp += 2;
    _regs.pc = ret_addr;
    _cycle_count += _bus->advance();
}

template<auto cond>
void Cpu::ret_cc() {
    if (cond(_regs.f)) {
        _cycle_count += _bus->advance();
        ret();
    } else {
        _cycle_count += _bus->advance();
    }
}

void Cpu::ld_hl_sp_s8() {
    _regs.f = 0x00;
    auto offset = static_cast<s8>(imm8());
    _cycle_count += _bus->advance();
    _regs.hl = _regs.sp + offset;
    if ((_regs.sp & 0xF) + (offset & 0xF) > 0xF) {
        _regs.f |= Alu::halfcarry;
    }
    if ((_regs.sp & 0xFF)  + (offset & 0xFF) > 0xFF) {
        _regs.f |= Alu::carry;
    }
}

void Cpu::ld_sp_hl() {
    _regs.sp = _regs.hl;
    _cycle_count += _bus->advance();
}

template<auto src>
void Cpu::push_r16() {
    _cycle_count += _bus->advance();
    push_sp_word(_regs.*src);
}

template<auto dst>
void Cpu::pop_r16() {
    u16 result = read_byte(_regs.sp++);
    result |= read_byte( _regs.sp++) << 8;
    _regs.*dst = result;
    _regs.f &= 0xF0;
}

void Cpu::adc_a_i8() {
    u8 value = imm8();
    u16 carry = (_regs.f & Alu::carry) ? 1 : 0;
    u16 res = _regs.a + value + carry;
    _regs.f = (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) + (value & 0x0F) + carry) > 0x0F) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

void Cpu::sbc_a_i8() {
    u8 value = imm8();
    u16 carry = (_regs.f & Alu::carry) ? 1 : 0;
    u16 res = _regs.a - value - carry;
    _regs.f = Alu::sub |
              (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) - (value & 0x0F) - carry) < 0) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

void Cpu::xor_a_i8() {
    u8 value = imm8();
    _regs.a ^= value;
    _regs.f = (_regs.a == 0) * Alu::zero;
}

void Cpu::cmp_a_i8() {
    u8 value = imm8();
    u8 a = _regs.a;
    _regs.f = Alu::sub;
    _regs.f |= (Alu::zero * (value == a));
    _regs.f |= (Alu::halfcarry * ((a & 0xF) < (value & 0xF)));
    _regs.f |= (Alu::carry * (a < value));
}

void Cpu::add_a_i8() {
    u8 value = imm8();
    u16 res = _regs.a + value;
    _regs.f = (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) + (value & 0x0F)) > 0x0F) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

void Cpu::sub_a_i8() {
    u8 value = imm8();
    u16 res = _regs.a - value;
    _regs.f = Alu::sub |
              (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) - (value & 0x0F)) < 0) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

void Cpu::and_a_i8() {
    u8 value = imm8();
    _regs.a &= value;
    _regs.f = Alu::halfcarry | ((_regs.a == 0) * Alu::zero);
}

void Cpu::or_a_i8() {
    u8 value = imm8();
    _regs.a |= value;
    _regs.f = (_regs.a == 0) * Alu::zero;
}

void Cpu::ld_a16_sp() {
    auto addr = imm16();
    write_byte(addr, _regs.sp & 0xFF);
    write_byte(addr+1, _regs.sp >> 8);
}

void Cpu::di() {
    _interrupts_enable = false;
}

void Cpu::ei() {
    _ime_delay = 1;
}

void Cpu::reti() {
    ret();
    ei();
}

u8 bit_set(u8 word, u8 bit_mask, bool value) {
    return (word & ~bit_mask) | (-value & bit_mask);
}

void Cpu::rlca(){
    bool carry = (_regs.a & 0x80) != 0;
    _regs.a = (_regs.a << 1) | (0x01 * carry);
    _regs.f = bit_set(0x00, Alu::carry, carry);
}

void Cpu::rrca(){
    bool carry = (_regs.a & 0x01) != 0;
    _regs.a = (_regs.a >> 1) | (0x80 * carry);
    _regs.f = bit_set(0x00, Alu::carry, carry);
}

void Cpu::rla(){
    bool bit7 = (_regs.a & 0x80) != 0;
    bool carry = (_regs.f & Alu::carry) != 0;

    _regs.a = (_regs.a << 1) | (0x01 * carry);
    _regs.f = (Alu::carry * bit7); 
}

void Cpu::rra(){
    bool bit1 = (_regs.a & 0x01) != 0;
    bool carry = (_regs.f & Alu::carry) != 0;

    _regs.a = (_regs.a >> 1) | (carry * 0x80);
    _regs.f = (Alu::carry * bit1);
}

void Cpu::cpl() {
    _regs.a ^= 0xFF;
    _regs.f |= Alu::halfcarry| Alu::sub;
}

void Cpu::ccf() {
    _regs.f ^= Alu::carry;
    _regs.f &= ~(Alu::halfcarry | Alu::sub);
}

void Cpu::scf() {
    _regs.f |= Alu::carry;
    _regs.f &= ~(Alu::halfcarry | Alu::sub);
}

void Cpu::inc_dhl() {
    auto val = read_byte(_regs.hl) + 1;
    write_byte(_regs.hl, val);

    _regs.f &= ~(Alu::sub | Alu::zero | Alu::halfcarry);
    if ((val & 0x0F) == 0) {
        _regs.f |= Alu::halfcarry;
    }

    if ((val & 0xFF) == 0) {
        _regs.f |= Alu::zero;
    }
}

void Cpu::dec_dhl() {
    auto val = read_byte(_regs.hl) - 1;
    write_byte(_regs.hl, val);

    _regs.f &= ~(Alu::zero | Alu::halfcarry);
    _regs.f |= Alu::sub;
    if ((val & 0x0F) == 0x0F) {
        _regs.f |= Alu::halfcarry;
    }

    if ((val & 0xFF) == 0) {
        _regs.f |= Alu::zero;
    }
}

void Cpu::stop() {
    imm8();
    _bus->write_byte(0xFF04, 0x00);
    _stopped = true;
}

void Cpu::halt() {
    if (_interrupts_enable) {
        _halted = true;
    } else {
        if (pending_interrupts()) {
            _regs.pc--;
        }
        _halted = !pending_interrupts();
    }
}

void Cpu::daa() {
    s16 result = _regs.a;
    _regs.f &= ~Alu::zero;
    if (_regs.f & Alu::sub) {
        if (_regs.f & Alu::halfcarry) {
            result = (result - 0x06) & 0xFF;
        }
        if (_regs.f & Alu::carry) {
            result -= 0x60;
        }
    }
    else {
        if ((_regs.f & Alu::halfcarry) || (result & 0x0F) > 0x09) {
            result += 0x06;
        }
        if ((_regs.f & Alu::carry) || result > 0x9F) {
            result += 0x60;
        }
    }
    if ((result & 0xFF) == 0) {
        _regs.f |= Alu::zero;
    }
    if ((result & 0x100) == 0x100) {
        _regs.f |= Alu::carry;
    }
    _regs.f &= ~Alu::halfcarry;
    _regs.a = result & 0xFF;
}

void Cpu::ld_dhl_i8() {
    write_byte(_regs.hl, imm8());
}

void Cpu::add_sp_s8() {
    u16 old_sp = _regs.sp;
    s16 offset = static_cast<s8>(imm8());
    _cycle_count += _bus->advance();
    _cycle_count += _bus->advance();
    _regs.sp += offset;

    _regs.f = 0;
    _regs.f |= (Alu::halfcarry * ((old_sp & 0xF) + (offset & 0xF) > 0xF));
    _regs.f |= Alu::carry * (((old_sp & 0xFF) + (offset & 0xFF) > 0xFF));
}

void Cpu::add_a_dhl() {
    u8 src = read_byte(_regs.hl);
    u16 res = _regs.a + src;
    _regs.f = (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) + (src & 0x0F)) > 0x0F) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

void Cpu::adc_a_dhl() {
    u8 src = read_byte(_regs.hl);
    u16 carry = (_regs.f & Alu::carry) ? 1 : 0;
    u16 res = _regs.a + src + carry;
    _regs.f = (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) + (src & 0x0F) + carry) > 0x0F) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

void Cpu::sub_a_dhl() {
    u8 src = read_byte(_regs.hl);
    u16 res = _regs.a - src;
    _regs.f = Alu::sub |
              (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) - (src & 0x0F)) < 0) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

void Cpu::sbc_a_dhl() {
    u8 src = read_byte(_regs.hl);
    u16 carry = (_regs.f & Alu::carry) ? 1 : 0;
    u16 res = _regs.a - src - carry;
    _regs.f = Alu::sub |
              (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) - (src & 0x0F) - carry) < 0) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
    _regs.a = res;
}

void Cpu::and_a_dhl() {
    _regs.a &= read_byte(_regs.hl);
    _regs.f = Alu::halfcarry | ((_regs.a == 0) * Alu::zero);
}

void Cpu::xor_a_dhl() {
    _regs.a ^= read_byte(_regs.hl);
    _regs.f = (_regs.a == 0) * Alu::zero;
}

void Cpu::or_a_dhl() {
    _regs.a |= read_byte(_regs.hl);
    _regs.f = (_regs.a == 0) * Alu::zero;
}

void Cpu::cp_a_dhl() {
    u8 src = read_byte(_regs.hl);
    u16 res = _regs.a - src;
    _regs.f = Alu::sub |
              (((res & 0xFF) == 0) * Alu::zero) |
              ((((_regs.a & 0x0F) - (src & 0x0F)) < 0) * Alu::halfcarry) |
              ((res > 0xFF) * Alu::carry);
}

inline
s8 Cpu::signed_read_byte(u16 addr) {
    return static_cast<s8>(read_byte(addr));
}

u8 Cpu::read_boot_byte(u16 addr) {
    if (boot_rom_memory_range.has(addr)) {
        return _boot_rom->read_byte(addr);
    }
    return read_bus_byte(addr);
}

void Cpu::write_boot_byte(u16 addr, u8 data) {
    if ((addr == 0xFF50) && (data & 0x1)) {
        _read_byte = &Cpu::read_bus_byte;
        _write_byte = &Cpu::write_bus_byte;
        return;
    }
    if (boot_rom_memory_range.has(addr)) {
        return _boot_rom->write_byte(addr, data);
    }
    return write_bus_byte(addr, data);
}

u8 Cpu::read_bus_byte(u16 addr) {
    auto result = _bus->read_byte(addr);
    _cycle_count += _bus->advance();
    return result;
}

void Cpu::write_bus_byte(u16 addr, u8 data) {
    _bus->write_byte(addr, data);
    _cycle_count += _bus->advance();
    if (addr == int_flag_addr) {
        _just_wrote_if = true;
    }
}

inline
u8 Cpu::read_byte(u16 addr) {
    return std::invoke(_read_byte, this, addr);
}

inline
void Cpu::write_byte(u16 addr, u8 data) {
    return std::invoke(_write_byte, this, addr, data);
}

inline
u16 Cpu::read_word(u16 addr) {
    u16 result = read_byte(addr);
    result |= (read_byte(addr+1) << 8);
    return result;
}

inline
void Cpu::push_sp_word(u16 data) {
    write_byte(--_regs.sp, data >> 8);
    write_byte(--_regs.sp, data & 0xFF);
}

inline
bool Cpu::pending_interrupts() const noexcept {
    return (_bus->read_byte(int_enable_addr) & _bus->read_byte(int_flag_addr) & 0x1F) != 0;
}

namespace {

u16 interrupt_vector_addr(u8 int_bit) {
    return 0x0040 + (int_bit * 0x08);
};

}

bool Cpu::check_interrupts() {
    if ((!_interrupts_enable) || (_just_wrote_if) || (!pending_interrupts())) {
        return false;
    }

    u8 triggered = (read_byte(int_enable_addr) & read_byte(int_flag_addr)) & 0x1F;

    _halted = false;
    u8 int_bit = std::countr_zero(triggered);
    _interrupts_enable = false;
        
    write_byte(int_flag_addr, _bus->read_byte(int_flag_addr) & ~(1<<int_bit));
    push_sp_word(_regs.pc);

    _regs.pc = interrupt_vector_addr(int_bit);
    return true;
}

struct CpuInstrTable {

    static constexpr auto ld_a_i8 = &Cpu::ld_dst_i8<&RF::a>;
    static constexpr auto ld_b_i8 = &Cpu::ld_dst_i8<&RF::b>;
    static constexpr auto ld_c_i8 = &Cpu::ld_dst_i8<&RF::c>;
    static constexpr auto ld_d_i8 = &Cpu::ld_dst_i8<&RF::d>;
    static constexpr auto ld_e_i8 = &Cpu::ld_dst_i8<&RF::e>;
    static constexpr auto ld_h_i8 = &Cpu::ld_dst_i8<&RF::h>;
    static constexpr auto ld_l_i8 = &Cpu::ld_dst_i8<&RF::l>;

    static constexpr auto ld_b_c    = &Cpu::ld_dst_src<&RF::b, &RF::c>;
    static constexpr auto ld_b_d    = &Cpu::ld_dst_src<&RF::b, &RF::d>;
    static constexpr auto ld_b_e    = &Cpu::ld_dst_src<&RF::b, &RF::e>;
    static constexpr auto ld_b_h    = &Cpu::ld_dst_src<&RF::b, &RF::h>;
    static constexpr auto ld_b_l    = &Cpu::ld_dst_src<&RF::b, &RF::l>;
    static constexpr auto ld_b_a    = &Cpu::ld_dst_src<&RF::b, &RF::a>;

    static constexpr auto ld_c_b    = &Cpu::ld_dst_src<&RF::c, &RF::b>;
    static constexpr auto ld_c_d    = &Cpu::ld_dst_src<&RF::c, &RF::d>;
    static constexpr auto ld_c_e    = &Cpu::ld_dst_src<&RF::c, &RF::e>;
    static constexpr auto ld_c_h    = &Cpu::ld_dst_src<&RF::c, &RF::h>;
    static constexpr auto ld_c_l    = &Cpu::ld_dst_src<&RF::c, &RF::l>;
    static constexpr auto ld_c_a    = &Cpu::ld_dst_src<&RF::c, &RF::a>;

    static constexpr auto ld_d_b    = &Cpu::ld_dst_src<&RF::d, &RF::b>;
    static constexpr auto ld_d_c    = &Cpu::ld_dst_src<&RF::d, &RF::c>;
    static constexpr auto ld_d_e    = &Cpu::ld_dst_src<&RF::d, &RF::e>;
    static constexpr auto ld_d_h    = &Cpu::ld_dst_src<&RF::d, &RF::h>;
    static constexpr auto ld_d_l    = &Cpu::ld_dst_src<&RF::d, &RF::l>;
    static constexpr auto ld_d_a    = &Cpu::ld_dst_src<&RF::d, &RF::a>;

    static constexpr auto ld_e_b    = &Cpu::ld_dst_src<&RF::e, &RF::b>;
    static constexpr auto ld_e_c    = &Cpu::ld_dst_src<&RF::e, &RF::c>;
    static constexpr auto ld_e_d    = &Cpu::ld_dst_src<&RF::e, &RF::d>;
    static constexpr auto ld_e_h    = &Cpu::ld_dst_src<&RF::e, &RF::h>;
    static constexpr auto ld_e_l    = &Cpu::ld_dst_src<&RF::e, &RF::l>;
    static constexpr auto ld_e_a    = &Cpu::ld_dst_src<&RF::e, &RF::a>;

    static constexpr auto ld_h_b    = &Cpu::ld_dst_src<&RF::h, &RF::b>;
    static constexpr auto ld_h_c    = &Cpu::ld_dst_src<&RF::h, &RF::c>;
    static constexpr auto ld_h_d    = &Cpu::ld_dst_src<&RF::h, &RF::d>;
    static constexpr auto ld_h_e    = &Cpu::ld_dst_src<&RF::h, &RF::e>;
    static constexpr auto ld_h_l    = &Cpu::ld_dst_src<&RF::h, &RF::l>;
    static constexpr auto ld_h_a    = &Cpu::ld_dst_src<&RF::h, &RF::a>;

    static constexpr auto ld_l_b    = &Cpu::ld_dst_src<&RF::l, &RF::b>;
    static constexpr auto ld_l_c    = &Cpu::ld_dst_src<&RF::l, &RF::c>;
    static constexpr auto ld_l_d    = &Cpu::ld_dst_src<&RF::l, &RF::d>;
    static constexpr auto ld_l_e    = &Cpu::ld_dst_src<&RF::l, &RF::e>;
    static constexpr auto ld_l_h    = &Cpu::ld_dst_src<&RF::l, &RF::h>;
    static constexpr auto ld_l_a    = &Cpu::ld_dst_src<&RF::l, &RF::a>;

    static constexpr auto ld_a_b    = &Cpu::ld_dst_src<&RF::a, &RF::b>;
    static constexpr auto ld_a_c    = &Cpu::ld_dst_src<&RF::a, &RF::c>;
    static constexpr auto ld_a_d    = &Cpu::ld_dst_src<&RF::a, &RF::d>;
    static constexpr auto ld_a_e    = &Cpu::ld_dst_src<&RF::a, &RF::e>;
    static constexpr auto ld_a_h    = &Cpu::ld_dst_src<&RF::a, &RF::h>;
    static constexpr auto ld_a_l    = &Cpu::ld_dst_src<&RF::a, &RF::l>;

    static constexpr auto ld_dhl_a  = &Cpu::ld_dhl_src<&RF::a>;
    static constexpr auto ld_dhl_b  = &Cpu::ld_dhl_src<&RF::b>;
    static constexpr auto ld_dhl_c  = &Cpu::ld_dhl_src<&RF::c>;
    static constexpr auto ld_dhl_d  = &Cpu::ld_dhl_src<&RF::d>;
    static constexpr auto ld_dhl_e  = &Cpu::ld_dhl_src<&RF::e>;
    static constexpr auto ld_dhl_h  = &Cpu::ld_dhl_src<&RF::h>;
    static constexpr auto ld_dhl_l  = &Cpu::ld_dhl_src<&RF::l>;

    static constexpr auto ld_c_dhl  = &Cpu::ld_dst_dhl<&RF::c>;
    static constexpr auto ld_b_dhl  = &Cpu::ld_dst_dhl<&RF::b>;
    static constexpr auto ld_d_dhl  = &Cpu::ld_dst_dhl<&RF::d>;
    static constexpr auto ld_e_dhl  = &Cpu::ld_dst_dhl<&RF::e>;
    static constexpr auto ld_h_dhl  = &Cpu::ld_dst_dhl<&RF::h>;
    static constexpr auto ld_l_dhl  = &Cpu::ld_dst_dhl<&RF::l>;
    static constexpr auto ld_a_dhl  = &Cpu::ld_dst_dhl<&RF::a>;

    static constexpr auto inc_a = &Cpu::inc_dst8<&RF::a>;
    static constexpr auto inc_b = &Cpu::inc_dst8<&RF::b>;
    static constexpr auto inc_c = &Cpu::inc_dst8<&RF::c>;
    static constexpr auto inc_d = &Cpu::inc_dst8<&RF::d>;
    static constexpr auto inc_e = &Cpu::inc_dst8<&RF::e>;
    static constexpr auto inc_h = &Cpu::inc_dst8<&RF::h>;
    static constexpr auto inc_l = &Cpu::inc_dst8<&RF::l>;

    static constexpr auto dec_a = &Cpu::dec_dst8<&RF::a>;
    static constexpr auto dec_b = &Cpu::dec_dst8<&RF::b>;
    static constexpr auto dec_c = &Cpu::dec_dst8<&RF::c>;
    static constexpr auto dec_d = &Cpu::dec_dst8<&RF::d>;
    static constexpr auto dec_e = &Cpu::dec_dst8<&RF::e>;
    static constexpr auto dec_h = &Cpu::dec_dst8<&RF::h>;
    static constexpr auto dec_l = &Cpu::dec_dst8<&RF::l>;

    static constexpr auto inc_bc = &Cpu::inc_dst16<&RF::bc>;
    static constexpr auto inc_de = &Cpu::inc_dst16<&RF::de>;
    static constexpr auto inc_hl = &Cpu::inc_dst16<&RF::hl>;
    static constexpr auto inc_sp = &Cpu::inc_dst16<&RF::sp>;

    static constexpr auto dec_bc = &Cpu::dec_dst16<&RF::bc>;
    static constexpr auto dec_de = &Cpu::dec_dst16<&RF::de>;
    static constexpr auto dec_hl = &Cpu::dec_dst16<&RF::hl>;
    static constexpr auto dec_sp = &Cpu::dec_dst16<&RF::sp>;

    static constexpr auto add_a_b = &Cpu::add_a_src<&RF::b>;
    static constexpr auto add_a_c = &Cpu::add_a_src<&RF::c>;
    static constexpr auto add_a_d = &Cpu::add_a_src<&RF::d>;
    static constexpr auto add_a_e = &Cpu::add_a_src<&RF::e>;
    static constexpr auto add_a_h = &Cpu::add_a_src<&RF::h>;
    static constexpr auto add_a_l = &Cpu::add_a_src<&RF::l>;
    static constexpr auto add_a_a = &Cpu::add_a_src<&RF::a>;

    static constexpr auto adc_a_b = &Cpu::adc_a_src<&RF::b>;
    static constexpr auto adc_a_c = &Cpu::adc_a_src<&RF::c>;
    static constexpr auto adc_a_d = &Cpu::adc_a_src<&RF::d>;
    static constexpr auto adc_a_e = &Cpu::adc_a_src<&RF::e>;
    static constexpr auto adc_a_h = &Cpu::adc_a_src<&RF::h>;
    static constexpr auto adc_a_l = &Cpu::adc_a_src<&RF::l>;
    static constexpr auto adc_a_a = &Cpu::adc_a_src<&RF::a>;

    static constexpr auto sub_a_b = &Cpu::sub_a_src<&RF::b>;
    static constexpr auto sub_a_c = &Cpu::sub_a_src<&RF::c>;
    static constexpr auto sub_a_d = &Cpu::sub_a_src<&RF::d>;
    static constexpr auto sub_a_e = &Cpu::sub_a_src<&RF::e>;
    static constexpr auto sub_a_h = &Cpu::sub_a_src<&RF::h>;
    static constexpr auto sub_a_l = &Cpu::sub_a_src<&RF::l>;
    static constexpr auto sub_a_a = &Cpu::sub_a_src<&RF::a>;

    static constexpr auto sbc_a_b = &Cpu::sbc_a_src<&RF::b>;
    static constexpr auto sbc_a_c = &Cpu::sbc_a_src<&RF::c>;
    static constexpr auto sbc_a_d = &Cpu::sbc_a_src<&RF::d>;
    static constexpr auto sbc_a_e = &Cpu::sbc_a_src<&RF::e>;
    static constexpr auto sbc_a_h = &Cpu::sbc_a_src<&RF::h>;
    static constexpr auto sbc_a_l = &Cpu::sbc_a_src<&RF::l>;
    static constexpr auto sbc_a_a = &Cpu::sbc_a_src<&RF::a>;

    static constexpr auto and_a_b = &Cpu::and_dst_src<&RF::b>;
    static constexpr auto and_a_c = &Cpu::and_dst_src<&RF::c>;
    static constexpr auto and_a_d = &Cpu::and_dst_src<&RF::d>;
    static constexpr auto and_a_e = &Cpu::and_dst_src<&RF::e>;
    static constexpr auto and_a_h = &Cpu::and_dst_src<&RF::h>;
    static constexpr auto and_a_l = &Cpu::and_dst_src<&RF::l>;
    static constexpr auto and_a_a = &Cpu::and_dst_src<&RF::a>;

    static constexpr auto xor_a_b = &Cpu::xor_a_src<&RF::b>;
    static constexpr auto xor_a_c = &Cpu::xor_a_src<&RF::c>;
    static constexpr auto xor_a_d = &Cpu::xor_a_src<&RF::d>;
    static constexpr auto xor_a_e = &Cpu::xor_a_src<&RF::e>;
    static constexpr auto xor_a_h = &Cpu::xor_a_src<&RF::h>;
    static constexpr auto xor_a_l = &Cpu::xor_a_src<&RF::l>;
    static constexpr auto xor_a_a = &Cpu::xor_a_src<&RF::a>;

    static constexpr auto or_a_b = &Cpu::or_a_src<&RF::b>;
    static constexpr auto or_a_c = &Cpu::or_a_src<&RF::c>;
    static constexpr auto or_a_d = &Cpu::or_a_src<&RF::d>;
    static constexpr auto or_a_e = &Cpu::or_a_src<&RF::e>;
    static constexpr auto or_a_h = &Cpu::or_a_src<&RF::h>;
    static constexpr auto or_a_l = &Cpu::or_a_src<&RF::l>;
    static constexpr auto or_a_a = &Cpu::or_a_src<&RF::a>;

    static constexpr auto cp_a_b = &Cpu::cp_a_src<&RF::b>;
    static constexpr auto cp_a_c = &Cpu::cp_a_src<&RF::c>;
    static constexpr auto cp_a_d = &Cpu::cp_a_src<&RF::d>;
    static constexpr auto cp_a_e = &Cpu::cp_a_src<&RF::e>;
    static constexpr auto cp_a_h = &Cpu::cp_a_src<&RF::h>;
    static constexpr auto cp_a_l = &Cpu::cp_a_src<&RF::l>;
    static constexpr auto cp_a_a = &Cpu::cp_a_src<&RF::a>;

    static constexpr auto nop = &Cpu::nop;

    static constexpr auto jr_nz = &Cpu::jr_cc<[](u8 flag){ return !(flag & Alu::zero); }>;
    static constexpr auto jr_z = &Cpu::jr_cc<[](u8 flag){ return (flag & Alu::zero); }>;
    static constexpr auto jr_nc = &Cpu::jr_cc<[](u8 flag){ return !(flag & Alu::carry); }>;
    static constexpr auto jr_c = &Cpu::jr_cc<[](u8 flag){ return (flag & Alu::carry); }>;

    static constexpr auto ld_hi_a = &Cpu::ld_hi_a;
    static constexpr auto ld_a_hi = &Cpu::ld_a_hi;

    static constexpr auto ld_a_bc = &Cpu::ld_a_drr<&RF::bc>;
    static constexpr auto ld_a_de = &Cpu::ld_a_drr<&RF::de>;

    static constexpr auto ld_bc_i16 = &Cpu::ldr_i16<&RF::bc>;
    static constexpr auto ld_de_i16 = &Cpu::ldr_i16<&RF::de>;
    static constexpr auto ld_hl_i16 = &Cpu::ldr_i16<&RF::hl>;
    static constexpr auto ld_sp_i16 = &Cpu::ldr_i16<&RF::sp>;

    static constexpr auto ldhl_dec = &Cpu::ld_hldec;
    static constexpr auto ldhl_inc = &Cpu::ld_hlinc;
    static constexpr auto ld_a_hlinc = &Cpu::ld_a_hlinc;
    static constexpr auto ld_a_hldec = &Cpu::ld_a_hldec;
    static constexpr auto ldhic_a = &Cpu::ldhic_a;
    static constexpr auto lda_hic = &Cpu::lda_hic;
    static constexpr auto ld_di16_a = &Cpu::ld_di16_a;
    static constexpr auto ld_a_di16 = &Cpu::ld_a_di16;
    static constexpr auto ld_dhl_i8 = &Cpu::ld_dhl_i8;
    static constexpr auto ld_hl_sp_s8 = &Cpu::ld_hl_sp_s8;
    static constexpr auto ld_sp_hl = &Cpu::ld_sp_hl;
    
    static constexpr auto jp = &Cpu::jp;
    static constexpr auto jp_hl = &Cpu::jp_hl;
    static constexpr auto jp_nz = &Cpu::jp_nz;
    static constexpr auto jp_nc = &Cpu::jp_nc;
    static constexpr auto jp_z = &Cpu::jp_z;
    static constexpr auto jp_c = &Cpu::jp_c;

    static constexpr auto jr = &Cpu::jr;
    static constexpr auto call_r16 = &Cpu::call_r16;
    static constexpr auto call_nz = &Cpu::call_nz;
    static constexpr auto call_nc = &Cpu::call_nc;
    static constexpr auto call_z = &Cpu::call_z;
    static constexpr auto call_c = &Cpu::call_c;
    static constexpr auto ret = &Cpu::ret;
    static constexpr auto ret_nz = &Cpu::ret_cc<[](u8 flag){ return !(flag & Alu::zero); }>;
    static constexpr auto ret_z  = &Cpu::ret_cc<[](u8 flag){ return (flag & Alu::zero); }>;
    static constexpr auto ret_nc = &Cpu::ret_cc<[](u8 flag){ return !(flag & Alu::carry); }>;
    static constexpr auto ret_c  = &Cpu::ret_cc<[](u8 flag){ return (flag & Alu::carry); }>;

    static constexpr auto push_bc = &Cpu::push_r16<&RF::bc>;
    static constexpr auto push_de = &Cpu::push_r16<&RF::de>;
    static constexpr auto push_hl = &Cpu::push_r16<&RF::hl>;
    static constexpr auto push_af = &Cpu::push_r16<&RF::af>;
    
    static constexpr auto pop_bc = &Cpu::pop_r16<&RF::bc>;
    static constexpr auto pop_de = &Cpu::pop_r16<&RF::de>;
    static constexpr auto pop_hl = &Cpu::pop_r16<&RF::hl>;
    static constexpr auto pop_af = &Cpu::pop_r16<&RF::af>;

    static constexpr auto adc_a_i8 = &Cpu::adc_a_i8;
    static constexpr auto sbc_a_i8 = &Cpu::sbc_a_i8;
    static constexpr auto xor_a_i8 = &Cpu::xor_a_i8;
    static constexpr auto cmp_a_i8 = &Cpu::cmp_a_i8;

    static constexpr auto add_a_i8 = &Cpu::add_a_i8;
    static constexpr auto sub_a_i8 = &Cpu::sub_a_i8;
    static constexpr auto and_a_i8 = &Cpu::and_a_i8;
    static constexpr auto or_a_i8 = &Cpu::or_a_i8;

    static constexpr auto di = &Cpu::di;
    static constexpr auto ei = &Cpu::ei;
    static constexpr auto reti = &Cpu::reti;

    static constexpr auto rlca = &Cpu::rlca;
    static constexpr auto rrca = &Cpu::rrca;
    static constexpr auto rla = &Cpu::rla;
    static constexpr auto rra = &Cpu::rra;

    static constexpr auto add_a_dhl = &Cpu::add_a_dhl;
    static constexpr auto adc_a_dhl = &Cpu::adc_a_dhl;
    static constexpr auto sub_a_dhl = &Cpu::sub_a_dhl;
    static constexpr auto sbc_a_dhl = &Cpu::sbc_a_dhl;
    static constexpr auto and_a_dhl = &Cpu::and_a_dhl;
    static constexpr auto xor_a_dhl = &Cpu::xor_a_dhl;
    static constexpr auto or_a_dhl = &Cpu::or_a_dhl;
    static constexpr auto cp_a_dhl = &Cpu::cp_a_dhl;

    static constexpr auto ld_dbc_a = &Cpu::ld_dst_a<&RF::bc>;
    static constexpr auto ld_dde_a = &Cpu::ld_dst_a<&RF::de>;

    static constexpr auto ld_a16_sp = &Cpu::ld_a16_sp;

    static constexpr auto add_hl_bc = &Cpu::add_hl_src<&RF::bc>;
    static constexpr auto add_hl_de = &Cpu::add_hl_src<&RF::de>;
    static constexpr auto add_hl_hl = &Cpu::add_hl_src<&RF::hl>;
    static constexpr auto add_hl_sp = &Cpu::add_hl_src<&RF::sp>;
    static constexpr auto add_sp_s8 = &Cpu::add_sp_s8;

    static constexpr auto stop = &Cpu::stop;
    static constexpr auto halt = &Cpu::halt;

    static constexpr auto daa = &Cpu::daa;

    static constexpr auto cpl = &Cpu::cpl;
    static constexpr auto ccf = &Cpu::ccf;
    static constexpr auto scf = &Cpu::scf;

    static constexpr auto inc_dhl = &Cpu::inc_dhl;
    static constexpr auto dec_dhl = &Cpu::dec_dhl;

    static constexpr auto mode_cb = &Cpu::mode_cb;

    static constexpr auto rst_0 = &Cpu::rst<0x0000>;
    static constexpr auto rst_1 = &Cpu::rst<0x0008>;
    static constexpr auto rst_2 = &Cpu::rst<0x0010>;
    static constexpr auto rst_3 = &Cpu::rst<0x0018>;
    static constexpr auto rst_4 = &Cpu::rst<0x0020>;
    static constexpr auto rst_5 = &Cpu::rst<0x0028>;
    static constexpr auto rst_6 = &Cpu::rst<0x0030>;
    static constexpr auto rst_7 = &Cpu::rst<0x0038>;

    static constexpr auto hcfCB = &Cpu::hcf<0xCB>;
    static constexpr auto hcfD3 = &Cpu::hcf<0xD3>;
    static constexpr auto hcfE3 = &Cpu::hcf<0xE3>;
    static constexpr auto hcfE4 = &Cpu::hcf<0xE4>;
    static constexpr auto hcfF4 = &Cpu::hcf<0xF4>;
    static constexpr auto hcfDB = &Cpu::hcf<0xDB>;
    static constexpr auto hcfEB = &Cpu::hcf<0xEB>;
    static constexpr auto hcfEC = &Cpu::hcf<0xEC>;
    static constexpr auto hcfFC = &Cpu::hcf<0xFC>;
    static constexpr auto hcfDD = &Cpu::hcf<0xDD>;
    static constexpr auto hcfED = &Cpu::hcf<0xED>;
    static constexpr auto hcfFD = &Cpu::hcf<0xFD>;

    static constexpr Cpu::StepFn base[] = {
    //     x0        x1        x2         x3        x4        x5         x6          x7        x8           x9          xA          xB         xC        xD        xE         xF
    /*0x*/ nop,      ld_bc_i16,  ld_dbc_a,  inc_bc,   inc_b,    dec_b,     ld_b_i8,    rlca,     ld_a16_sp,   add_hl_bc,  ld_a_bc,   dec_bc,    inc_c,    dec_c,    ld_c_i8,   rrca,
    /*1x*/ stop,     ld_de_i16,  ld_dde_a,  inc_de,   inc_d,    dec_d,     ld_d_i8,    rla,      jr,          add_hl_de,  ld_a_de,   dec_de,    inc_e,    dec_e,    ld_e_i8,   rra,
    /*2x*/ jr_nz,    ld_hl_i16,  ldhl_inc,  inc_hl,   inc_h,    dec_h,     ld_h_i8,    daa,      jr_z,       add_hl_hl,  ld_a_hlinc, dec_hl,    inc_l,    dec_l,    ld_l_i8,   cpl,
    /*3x*/ jr_nc,    ld_sp_i16,  ldhl_dec,  inc_sp,   inc_dhl,  dec_dhl,   ld_dhl_i8,  scf,      jr_c,       add_hl_sp,  ld_a_hldec, dec_sp,    inc_a,    dec_a,    ld_a_i8,   ccf,
    /*4x*/ nop,      ld_b_c,   ld_b_d,    ld_b_e,   ld_b_h,   ld_b_l,    ld_b_dhl,   ld_b_a,   ld_c_b,      nop,        ld_c_d,     ld_c_e,    ld_c_h,   ld_c_l,   ld_c_dhl,  ld_c_a,
    /*5x*/ ld_d_b,   ld_d_c,   nop,       ld_d_e,   ld_d_h,   ld_d_l,    ld_d_dhl,   ld_d_a,   ld_e_b,      ld_e_c,     ld_e_d,     nop,       ld_e_h,   ld_e_l,   ld_e_dhl,  ld_e_a,
    /*6x*/ ld_h_b,   ld_h_c,   ld_h_d,    ld_h_e,   nop,      ld_h_l,    ld_h_dhl,   ld_h_a,   ld_l_b,      ld_l_c,     ld_l_d,     ld_l_e,    ld_l_h,   nop,      ld_l_dhl,  ld_l_a,
    /*7x*/ ld_dhl_b, ld_dhl_c, ld_dhl_d,  ld_dhl_e, ld_dhl_h, ld_dhl_l,  halt,       ld_dhl_a, ld_a_b,      ld_a_c,     ld_a_d,     ld_a_e,    ld_a_h,   ld_a_l,   ld_a_dhl,  nop,
    /*8x*/ add_a_b,  add_a_c,  add_a_d,   add_a_e,  add_a_h,  add_a_l,   add_a_dhl,  add_a_a,  adc_a_b,     adc_a_c,    adc_a_d,    adc_a_e,   adc_a_h,  adc_a_l,  adc_a_dhl, adc_a_a,
    /*9x*/ sub_a_b,  sub_a_c,  sub_a_d,   sub_a_e,  sub_a_h,  sub_a_l,   sub_a_dhl,  sub_a_a,  sbc_a_b,     sbc_a_c,    sbc_a_d,    sbc_a_e,   sbc_a_h,  sbc_a_l,  sbc_a_dhl, sbc_a_a,
    /*Ax*/ and_a_b,  and_a_c,  and_a_d,   and_a_e,  and_a_h,  and_a_l,   and_a_dhl,  and_a_a,  xor_a_b,     xor_a_c,    xor_a_d,    xor_a_e,   xor_a_h,  xor_a_l,  xor_a_dhl, xor_a_a,
    /*Bx*/ or_a_b,   or_a_c,   or_a_d,    or_a_e,   or_a_h,   or_a_l,    or_a_dhl,   or_a_a,   cp_a_b,      cp_a_c,     cp_a_d,     cp_a_e,    cp_a_h,   cp_a_l,   cp_a_dhl,  cp_a_a,
    /*Cx*/ ret_nz,   pop_bc,  jp_nz,     jp,       call_nz,  push_bc,  add_a_i8,   rst_0,    ret_z,       ret,        jp_z,       mode_cb,    call_z,   call_r16, adc_a_i8,  rst_1,
    /*Dx*/ ret_nc,   pop_de,  jp_nc,     hcfD3,    call_nc,  push_de,  sub_a_i8,   rst_2,    ret_c,       reti,       jp_c,       hcfDB,     call_c,   hcfDD,    sbc_a_i8,  rst_3,
    /*Ex*/ ld_hi_a,  pop_hl,  ldhic_a,   hcfE3,    hcfE4,    push_hl,  and_a_i8,   rst_4,    add_sp_s8,   jp_hl,      ld_di16_a,  hcfEB,     hcfEC,    hcfED,    xor_a_i8,  rst_5,
    /*Fx*/ ld_a_hi,  pop_af,  lda_hic,   di,       hcfF4,    push_af,  or_a_i8,    rst_6,    ld_hl_sp_s8, ld_sp_hl,   ld_a_di16,  ei,        hcfFC,    hcfFD,    cmp_a_i8,  rst_7
    };

    static_assert(std::size(base) == 0x100, "lost some opcodes");

    static constexpr auto rlc_b = &Cpu::cb_rlc<&RF::b>;
    static constexpr auto rlc_c = &Cpu::cb_rlc<&RF::c>;
    static constexpr auto rlc_d = &Cpu::cb_rlc<&RF::d>;
    static constexpr auto rlc_e = &Cpu::cb_rlc<&RF::e>;
    static constexpr auto rlc_h = &Cpu::cb_rlc<&RF::h>;
    static constexpr auto rlc_l = &Cpu::cb_rlc<&RF::l>;
    static constexpr auto rlc_a = &Cpu::cb_rlc<&RF::a>;

    static constexpr auto rrc_b = &Cpu::cb_rrc<&RF::b>;
    static constexpr auto rrc_c = &Cpu::cb_rrc<&RF::c>;
    static constexpr auto rrc_d = &Cpu::cb_rrc<&RF::d>;
    static constexpr auto rrc_e = &Cpu::cb_rrc<&RF::e>;
    static constexpr auto rrc_h = &Cpu::cb_rrc<&RF::h>;
    static constexpr auto rrc_l = &Cpu::cb_rrc<&RF::l>;
    static constexpr auto rrc_a = &Cpu::cb_rrc<&RF::a>;

    static constexpr auto rl_b = &Cpu::cb_rl<&RF::b>;
    static constexpr auto rl_c = &Cpu::cb_rl<&RF::c>;
    static constexpr auto rl_d = &Cpu::cb_rl<&RF::d>;
    static constexpr auto rl_e = &Cpu::cb_rl<&RF::e>;
    static constexpr auto rl_h = &Cpu::cb_rl<&RF::h>;
    static constexpr auto rl_l = &Cpu::cb_rl<&RF::l>;
    static constexpr auto rl_a = &Cpu::cb_rl<&RF::a>;

    static constexpr auto rr_b = &Cpu::cb_rr<&RF::b>;
    static constexpr auto rr_c = &Cpu::cb_rr<&RF::c>;
    static constexpr auto rr_d = &Cpu::cb_rr<&RF::d>;
    static constexpr auto rr_e = &Cpu::cb_rr<&RF::e>;
    static constexpr auto rr_h = &Cpu::cb_rr<&RF::h>;
    static constexpr auto rr_l = &Cpu::cb_rr<&RF::l>;
    static constexpr auto rr_a = &Cpu::cb_rr<&RF::a>;

    static constexpr auto sla_b = &Cpu::cb_sla<&RF::b>;
    static constexpr auto sla_c = &Cpu::cb_sla<&RF::c>;
    static constexpr auto sla_d = &Cpu::cb_sla<&RF::d>;
    static constexpr auto sla_e = &Cpu::cb_sla<&RF::e>;
    static constexpr auto sla_h = &Cpu::cb_sla<&RF::h>;
    static constexpr auto sla_l = &Cpu::cb_sla<&RF::l>;
    static constexpr auto sla_a = &Cpu::cb_sla<&RF::a>;

    static constexpr auto sra_b = &Cpu::cb_sra<&RF::b>;
    static constexpr auto sra_c = &Cpu::cb_sra<&RF::c>;
    static constexpr auto sra_d = &Cpu::cb_sra<&RF::d>;
    static constexpr auto sra_e = &Cpu::cb_sra<&RF::e>;
    static constexpr auto sra_h = &Cpu::cb_sra<&RF::h>;
    static constexpr auto sra_l = &Cpu::cb_sra<&RF::l>;
    static constexpr auto sra_a = &Cpu::cb_sra<&RF::a>;

    static constexpr auto swap_b = &Cpu::cb_swap<&RF::b>;
    static constexpr auto swap_c = &Cpu::cb_swap<&RF::c>;
    static constexpr auto swap_d = &Cpu::cb_swap<&RF::d>;
    static constexpr auto swap_e = &Cpu::cb_swap<&RF::e>;
    static constexpr auto swap_h = &Cpu::cb_swap<&RF::h>;
    static constexpr auto swap_l = &Cpu::cb_swap<&RF::l>;
    static constexpr auto swap_a = &Cpu::cb_swap<&RF::a>;

    static constexpr auto srl_b = &Cpu::cb_srl<&RF::b>;
    static constexpr auto srl_c = &Cpu::cb_srl<&RF::c>;
    static constexpr auto srl_d = &Cpu::cb_srl<&RF::d>;
    static constexpr auto srl_e = &Cpu::cb_srl<&RF::e>;
    static constexpr auto srl_h = &Cpu::cb_srl<&RF::h>;
    static constexpr auto srl_l = &Cpu::cb_srl<&RF::l>;
    static constexpr auto srl_a = &Cpu::cb_srl<&RF::a>;

    static constexpr auto bit_0_b = &Cpu::cb_test<&RF::b, 0x01>;
    static constexpr auto bit_0_c = &Cpu::cb_test<&RF::c, 0x01>;
    static constexpr auto bit_0_d = &Cpu::cb_test<&RF::d, 0x01>;
    static constexpr auto bit_0_e = &Cpu::cb_test<&RF::e, 0x01>;
    static constexpr auto bit_0_h = &Cpu::cb_test<&RF::h, 0x01>;
    static constexpr auto bit_0_l = &Cpu::cb_test<&RF::l, 0x01>;
    static constexpr auto bit_0_a = &Cpu::cb_test<&RF::a, 0x01>;

    static constexpr auto bit_1_b = &Cpu::cb_test<&RF::b, 0x02>;
    static constexpr auto bit_1_c = &Cpu::cb_test<&RF::c, 0x02>;
    static constexpr auto bit_1_d = &Cpu::cb_test<&RF::d, 0x02>;
    static constexpr auto bit_1_e = &Cpu::cb_test<&RF::e, 0x02>;
    static constexpr auto bit_1_h = &Cpu::cb_test<&RF::h, 0x02>;
    static constexpr auto bit_1_l = &Cpu::cb_test<&RF::l, 0x02>;
    static constexpr auto bit_1_a = &Cpu::cb_test<&RF::a, 0x02>;

    static constexpr auto bit_2_b = &Cpu::cb_test<&RF::b, 0x04>;
    static constexpr auto bit_2_c = &Cpu::cb_test<&RF::c, 0x04>;
    static constexpr auto bit_2_d = &Cpu::cb_test<&RF::d, 0x04>;
    static constexpr auto bit_2_e = &Cpu::cb_test<&RF::e, 0x04>;
    static constexpr auto bit_2_h = &Cpu::cb_test<&RF::h, 0x04>;
    static constexpr auto bit_2_l = &Cpu::cb_test<&RF::l, 0x04>;
    static constexpr auto bit_2_a = &Cpu::cb_test<&RF::a, 0x04>;

    static constexpr auto bit_3_b = &Cpu::cb_test<&RF::b, 0x08>;
    static constexpr auto bit_3_c = &Cpu::cb_test<&RF::c, 0x08>;
    static constexpr auto bit_3_d = &Cpu::cb_test<&RF::d, 0x08>;
    static constexpr auto bit_3_e = &Cpu::cb_test<&RF::e, 0x08>;
    static constexpr auto bit_3_h = &Cpu::cb_test<&RF::h, 0x08>;
    static constexpr auto bit_3_l = &Cpu::cb_test<&RF::l, 0x08>;
    static constexpr auto bit_3_a = &Cpu::cb_test<&RF::a, 0x08>;

    static constexpr auto bit_4_b = &Cpu::cb_test<&RF::b, 0x10>;
    static constexpr auto bit_4_c = &Cpu::cb_test<&RF::c, 0x10>;
    static constexpr auto bit_4_d = &Cpu::cb_test<&RF::d, 0x10>;
    static constexpr auto bit_4_e = &Cpu::cb_test<&RF::e, 0x10>;
    static constexpr auto bit_4_h = &Cpu::cb_test<&RF::h, 0x10>;
    static constexpr auto bit_4_l = &Cpu::cb_test<&RF::l, 0x10>;
    static constexpr auto bit_4_a = &Cpu::cb_test<&RF::a, 0x10>;

    static constexpr auto bit_5_b = &Cpu::cb_test<&RF::b, 0x20>;
    static constexpr auto bit_5_c = &Cpu::cb_test<&RF::c, 0x20>;
    static constexpr auto bit_5_d = &Cpu::cb_test<&RF::d, 0x20>;
    static constexpr auto bit_5_e = &Cpu::cb_test<&RF::e, 0x20>;
    static constexpr auto bit_5_h = &Cpu::cb_test<&RF::h, 0x20>;
    static constexpr auto bit_5_l = &Cpu::cb_test<&RF::l, 0x20>;
    static constexpr auto bit_5_a = &Cpu::cb_test<&RF::a, 0x20>;

    static constexpr auto bit_6_b = &Cpu::cb_test<&RF::b, 0x40>;
    static constexpr auto bit_6_c = &Cpu::cb_test<&RF::c, 0x40>;
    static constexpr auto bit_6_d = &Cpu::cb_test<&RF::d, 0x40>;
    static constexpr auto bit_6_e = &Cpu::cb_test<&RF::e, 0x40>;
    static constexpr auto bit_6_h = &Cpu::cb_test<&RF::h, 0x40>;
    static constexpr auto bit_6_l = &Cpu::cb_test<&RF::l, 0x40>;
    static constexpr auto bit_6_a = &Cpu::cb_test<&RF::a, 0x40>;

    static constexpr auto bit_7_b = &Cpu::cb_test<&RF::b, 0x80>;
    static constexpr auto bit_7_c = &Cpu::cb_test<&RF::c, 0x80>;
    static constexpr auto bit_7_d = &Cpu::cb_test<&RF::d, 0x80>;
    static constexpr auto bit_7_e = &Cpu::cb_test<&RF::e, 0x80>;
    static constexpr auto bit_7_h = &Cpu::cb_test<&RF::h, 0x80>;
    static constexpr auto bit_7_l = &Cpu::cb_test<&RF::l, 0x80>;
    static constexpr auto bit_7_a = &Cpu::cb_test<&RF::a, 0x80>;

    static constexpr auto res_0_b = &Cpu::cb_reset<&RF::b, 0x01>;
    static constexpr auto res_0_c = &Cpu::cb_reset<&RF::c, 0x01>;
    static constexpr auto res_0_d = &Cpu::cb_reset<&RF::d, 0x01>;
    static constexpr auto res_0_e = &Cpu::cb_reset<&RF::e, 0x01>;
    static constexpr auto res_0_h = &Cpu::cb_reset<&RF::h, 0x01>;
    static constexpr auto res_0_l = &Cpu::cb_reset<&RF::l, 0x01>;
    static constexpr auto res_0_a = &Cpu::cb_reset<&RF::a, 0x01>;

    static constexpr auto res_1_b = &Cpu::cb_reset<&RF::b, 0x02>;
    static constexpr auto res_1_c = &Cpu::cb_reset<&RF::c, 0x02>;
    static constexpr auto res_1_d = &Cpu::cb_reset<&RF::d, 0x02>;
    static constexpr auto res_1_e = &Cpu::cb_reset<&RF::e, 0x02>;
    static constexpr auto res_1_h = &Cpu::cb_reset<&RF::h, 0x02>;
    static constexpr auto res_1_l = &Cpu::cb_reset<&RF::l, 0x02>;
    static constexpr auto res_1_a = &Cpu::cb_reset<&RF::a, 0x02>;

    static constexpr auto res_2_b = &Cpu::cb_reset<&RF::b, 0x04>;
    static constexpr auto res_2_c = &Cpu::cb_reset<&RF::c, 0x04>;
    static constexpr auto res_2_d = &Cpu::cb_reset<&RF::d, 0x04>;
    static constexpr auto res_2_e = &Cpu::cb_reset<&RF::e, 0x04>;
    static constexpr auto res_2_h = &Cpu::cb_reset<&RF::h, 0x04>;
    static constexpr auto res_2_l = &Cpu::cb_reset<&RF::l, 0x04>;
    static constexpr auto res_2_a = &Cpu::cb_reset<&RF::a, 0x04>;

    static constexpr auto res_3_b = &Cpu::cb_reset<&RF::b, 0x08>;
    static constexpr auto res_3_c = &Cpu::cb_reset<&RF::c, 0x08>;
    static constexpr auto res_3_d = &Cpu::cb_reset<&RF::d, 0x08>;
    static constexpr auto res_3_e = &Cpu::cb_reset<&RF::e, 0x08>;
    static constexpr auto res_3_h = &Cpu::cb_reset<&RF::h, 0x08>;
    static constexpr auto res_3_l = &Cpu::cb_reset<&RF::l, 0x08>;
    static constexpr auto res_3_a = &Cpu::cb_reset<&RF::a, 0x08>;

    static constexpr auto res_4_b = &Cpu::cb_reset<&RF::b, 0x10>;
    static constexpr auto res_4_c = &Cpu::cb_reset<&RF::c, 0x10>;
    static constexpr auto res_4_d = &Cpu::cb_reset<&RF::d, 0x10>;
    static constexpr auto res_4_e = &Cpu::cb_reset<&RF::e, 0x10>;
    static constexpr auto res_4_h = &Cpu::cb_reset<&RF::h, 0x10>;
    static constexpr auto res_4_l = &Cpu::cb_reset<&RF::l, 0x10>;
    static constexpr auto res_4_a = &Cpu::cb_reset<&RF::a, 0x10>;

    static constexpr auto res_5_b = &Cpu::cb_reset<&RF::b, 0x20>;
    static constexpr auto res_5_c = &Cpu::cb_reset<&RF::c, 0x20>;
    static constexpr auto res_5_d = &Cpu::cb_reset<&RF::d, 0x20>;
    static constexpr auto res_5_e = &Cpu::cb_reset<&RF::e, 0x20>;
    static constexpr auto res_5_h = &Cpu::cb_reset<&RF::h, 0x20>;
    static constexpr auto res_5_l = &Cpu::cb_reset<&RF::l, 0x20>;
    static constexpr auto res_5_a = &Cpu::cb_reset<&RF::a, 0x20>;

    static constexpr auto res_6_b = &Cpu::cb_reset<&RF::b, 0x40>;
    static constexpr auto res_6_c = &Cpu::cb_reset<&RF::c, 0x40>;
    static constexpr auto res_6_d = &Cpu::cb_reset<&RF::d, 0x40>;
    static constexpr auto res_6_e = &Cpu::cb_reset<&RF::e, 0x40>;
    static constexpr auto res_6_h = &Cpu::cb_reset<&RF::h, 0x40>;
    static constexpr auto res_6_l = &Cpu::cb_reset<&RF::l, 0x40>;
    static constexpr auto res_6_a = &Cpu::cb_reset<&RF::a, 0x40>;

    static constexpr auto res_7_b = &Cpu::cb_reset<&RF::b, 0x80>;
    static constexpr auto res_7_c = &Cpu::cb_reset<&RF::c, 0x80>;
    static constexpr auto res_7_d = &Cpu::cb_reset<&RF::d, 0x80>;
    static constexpr auto res_7_e = &Cpu::cb_reset<&RF::e, 0x80>;
    static constexpr auto res_7_h = &Cpu::cb_reset<&RF::h, 0x80>;
    static constexpr auto res_7_l = &Cpu::cb_reset<&RF::l, 0x80>;
    static constexpr auto res_7_a = &Cpu::cb_reset<&RF::a, 0x80>;

    static constexpr auto set_0_b = &Cpu::cb_set<&RF::b, 0x01>;
    static constexpr auto set_0_c = &Cpu::cb_set<&RF::c, 0x01>;
    static constexpr auto set_0_d = &Cpu::cb_set<&RF::d, 0x01>;
    static constexpr auto set_0_e = &Cpu::cb_set<&RF::e, 0x01>;
    static constexpr auto set_0_h = &Cpu::cb_set<&RF::h, 0x01>;
    static constexpr auto set_0_l = &Cpu::cb_set<&RF::l, 0x01>;
    static constexpr auto set_0_a = &Cpu::cb_set<&RF::a, 0x01>;

    static constexpr auto set_1_b = &Cpu::cb_set<&RF::b, 0x02>;
    static constexpr auto set_1_c = &Cpu::cb_set<&RF::c, 0x02>;
    static constexpr auto set_1_d = &Cpu::cb_set<&RF::d, 0x02>;
    static constexpr auto set_1_e = &Cpu::cb_set<&RF::e, 0x02>;
    static constexpr auto set_1_h = &Cpu::cb_set<&RF::h, 0x02>;
    static constexpr auto set_1_l = &Cpu::cb_set<&RF::l, 0x02>;
    static constexpr auto set_1_a = &Cpu::cb_set<&RF::a, 0x02>;

    static constexpr auto set_2_b = &Cpu::cb_set<&RF::b, 0x04>;
    static constexpr auto set_2_c = &Cpu::cb_set<&RF::c, 0x04>;
    static constexpr auto set_2_d = &Cpu::cb_set<&RF::d, 0x04>;
    static constexpr auto set_2_e = &Cpu::cb_set<&RF::e, 0x04>;
    static constexpr auto set_2_h = &Cpu::cb_set<&RF::h, 0x04>;
    static constexpr auto set_2_l = &Cpu::cb_set<&RF::l, 0x04>;
    static constexpr auto set_2_a = &Cpu::cb_set<&RF::a, 0x04>;

    static constexpr auto set_3_b = &Cpu::cb_set<&RF::b, 0x08>;
    static constexpr auto set_3_c = &Cpu::cb_set<&RF::c, 0x08>;
    static constexpr auto set_3_d = &Cpu::cb_set<&RF::d, 0x08>;
    static constexpr auto set_3_e = &Cpu::cb_set<&RF::e, 0x08>;
    static constexpr auto set_3_h = &Cpu::cb_set<&RF::h, 0x08>;
    static constexpr auto set_3_l = &Cpu::cb_set<&RF::l, 0x08>;
    static constexpr auto set_3_a = &Cpu::cb_set<&RF::a, 0x08>;

    static constexpr auto set_4_b = &Cpu::cb_set<&RF::b, 0x10>;
    static constexpr auto set_4_c = &Cpu::cb_set<&RF::c, 0x10>;
    static constexpr auto set_4_d = &Cpu::cb_set<&RF::d, 0x10>;
    static constexpr auto set_4_e = &Cpu::cb_set<&RF::e, 0x10>;
    static constexpr auto set_4_h = &Cpu::cb_set<&RF::h, 0x10>;
    static constexpr auto set_4_l = &Cpu::cb_set<&RF::l, 0x10>;
    static constexpr auto set_4_a = &Cpu::cb_set<&RF::a, 0x10>;

    static constexpr auto set_5_b = &Cpu::cb_set<&RF::b, 0x20>;
    static constexpr auto set_5_c = &Cpu::cb_set<&RF::c, 0x20>;
    static constexpr auto set_5_d = &Cpu::cb_set<&RF::d, 0x20>;
    static constexpr auto set_5_e = &Cpu::cb_set<&RF::e, 0x20>;
    static constexpr auto set_5_h = &Cpu::cb_set<&RF::h, 0x20>;
    static constexpr auto set_5_l = &Cpu::cb_set<&RF::l, 0x20>;
    static constexpr auto set_5_a = &Cpu::cb_set<&RF::a, 0x20>;

    static constexpr auto set_6_b = &Cpu::cb_set<&RF::b, 0x40>;
    static constexpr auto set_6_c = &Cpu::cb_set<&RF::c, 0x40>;
    static constexpr auto set_6_d = &Cpu::cb_set<&RF::d, 0x40>;
    static constexpr auto set_6_e = &Cpu::cb_set<&RF::e, 0x40>;
    static constexpr auto set_6_h = &Cpu::cb_set<&RF::h, 0x40>;
    static constexpr auto set_6_l = &Cpu::cb_set<&RF::l, 0x40>;
    static constexpr auto set_6_a = &Cpu::cb_set<&RF::a, 0x40>;

    static constexpr auto set_7_b = &Cpu::cb_set<&RF::b, 0x80>;
    static constexpr auto set_7_c = &Cpu::cb_set<&RF::c, 0x80>;
    static constexpr auto set_7_d = &Cpu::cb_set<&RF::d, 0x80>;
    static constexpr auto set_7_e = &Cpu::cb_set<&RF::e, 0x80>;
    static constexpr auto set_7_h = &Cpu::cb_set<&RF::h, 0x80>;
    static constexpr auto set_7_l = &Cpu::cb_set<&RF::l, 0x80>;
    static constexpr auto set_7_a = &Cpu::cb_set<&RF::a, 0x80>;

    static constexpr auto rlc_dhl = &Cpu::cb_rlc_m;
    static constexpr auto rrc_dhl = &Cpu::cb_rrc_m;
    static constexpr auto rl_dhl = &Cpu::cb_rl_m;
    static constexpr auto rr_dhl = &Cpu::cb_rr_m;
    static constexpr auto sla_dhl = &Cpu::cb_sla_m;
    static constexpr auto sra_dhl = &Cpu::cb_sra_m;
    static constexpr auto swap_dhl = &Cpu::cb_swap_m;
    static constexpr auto srl_dhl = &Cpu::cb_srl_m;
    static constexpr auto bit_0_dhl = &Cpu::cb_test_m<0x01>;
    static constexpr auto bit_1_dhl = &Cpu::cb_test_m<0x02>;
    static constexpr auto bit_2_dhl = &Cpu::cb_test_m<0x04>;
    static constexpr auto bit_3_dhl = &Cpu::cb_test_m<0x08>;
    static constexpr auto bit_4_dhl = &Cpu::cb_test_m<0x10>;
    static constexpr auto bit_5_dhl = &Cpu::cb_test_m<0x20>;
    static constexpr auto bit_6_dhl = &Cpu::cb_test_m<0x40>;
    static constexpr auto bit_7_dhl = &Cpu::cb_test_m<0x80>;
    static constexpr auto res_0_dhl = &Cpu::cb_reset_m<0x01>;
    static constexpr auto res_1_dhl = &Cpu::cb_reset_m<0x02>;
    static constexpr auto res_2_dhl = &Cpu::cb_reset_m<0x04>;
    static constexpr auto res_3_dhl = &Cpu::cb_reset_m<0x08>;
    static constexpr auto res_4_dhl = &Cpu::cb_reset_m<0x10>;
    static constexpr auto res_5_dhl = &Cpu::cb_reset_m<0x20>;
    static constexpr auto res_6_dhl = &Cpu::cb_reset_m<0x40>;
    static constexpr auto res_7_dhl = &Cpu::cb_reset_m<0x80>;
    static constexpr auto set_0_dhl = &Cpu::cb_set_m<0x01>;
    static constexpr auto set_1_dhl = &Cpu::cb_set_m<0x02>;
    static constexpr auto set_2_dhl = &Cpu::cb_set_m<0x04>;
    static constexpr auto set_3_dhl = &Cpu::cb_set_m<0x08>;
    static constexpr auto set_4_dhl = &Cpu::cb_set_m<0x10>;
    static constexpr auto set_5_dhl = &Cpu::cb_set_m<0x20>;
    static constexpr auto set_6_dhl = &Cpu::cb_set_m<0x40>;
    static constexpr auto set_7_dhl = &Cpu::cb_set_m<0x80>;

    static constexpr Cpu::StepFn cb_map[] = {
    //     x0        x1        x2         x3        x4        x5         x6          x7        x8           x9          xA          xB         xC        xD        xE         xF
    /*0x*/ rlc_b,    rlc_c,    rlc_d,     rlc_e,    rlc_h,    rlc_l,     rlc_dhl,    rlc_a,    rrc_b,      rrc_c,      rrc_d,      rrc_e,     rrc_h,    rrc_l,    rrc_dhl,   rrc_a,
    /*1x*/ rl_b,     rl_c,     rl_d,      rl_e,     rl_h,     rl_l,      rl_dhl,     rl_a,     rr_b,       rr_c,       rr_d,       rr_e,      rr_h,     rr_l,     rr_dhl,    rr_a,
    /*2x*/ sla_b,    sla_c,    sla_d,     sla_e,    sla_h,    sla_l,     sla_dhl,    sla_a,    sra_b,      sra_c,      sra_d,      sra_e,     sra_h,    sra_l,    sra_dhl,   sra_a,
    /*3x*/ swap_b,   swap_c,   swap_d,    swap_e,   swap_h,   swap_l,    swap_dhl,   swap_a,   srl_b,      srl_c,      srl_d,      srl_e,     srl_h,    srl_l,    srl_dhl,   srl_a,
    /*4x*/ bit_0_b,  bit_0_c,  bit_0_d,   bit_0_e,  bit_0_h,  bit_0_l,   bit_0_dhl,  bit_0_a,  bit_1_b,    bit_1_c,    bit_1_d,    bit_1_e,   bit_1_h,  bit_1_l,  bit_1_dhl, bit_1_a,
    /*5x*/ bit_2_b,  bit_2_c,  bit_2_d,   bit_2_e,  bit_2_h,  bit_2_l,   bit_2_dhl,  bit_2_a,  bit_3_b,    bit_3_c,    bit_3_d,    bit_3_e,   bit_3_h,  bit_3_l,  bit_3_dhl, bit_3_a,
    /*6x*/ bit_4_b,  bit_4_c,  bit_4_d,   bit_4_e,  bit_4_h,  bit_4_l,   bit_4_dhl,  bit_4_a,  bit_5_b,    bit_5_c,    bit_5_d,    bit_5_e,   bit_5_h,  bit_5_l,  bit_5_dhl, bit_5_a,
    /*7x*/ bit_6_b,  bit_6_c,  bit_6_d,   bit_6_e,  bit_6_h,  bit_6_l,   bit_6_dhl,  bit_6_a,  bit_7_b,    bit_7_c,    bit_7_d,    bit_7_e,   bit_7_h,  bit_7_l,  bit_7_dhl, bit_7_a,
    /*8x*/ res_0_b,  res_0_c,  res_0_d,   res_0_e,  res_0_h,  res_0_l,   res_0_dhl,  res_0_a,  res_1_b,    res_1_c,    res_1_d,    res_1_e,   res_1_h,  res_1_l,  res_1_dhl, res_1_a,
    /*9x*/ res_2_b,  res_2_c,  res_2_d,   res_2_e,  res_2_h,  res_2_l,   res_2_dhl,  res_2_a,  res_3_b,    res_3_c,    res_3_d,    res_3_e,   res_3_h,  res_3_l,  res_3_dhl, res_3_a,
    /*Ax*/ res_4_b,  res_4_c,  res_4_d,   res_4_e,  res_4_h,  res_4_l,   res_4_dhl,  res_4_a,  res_5_b,    res_5_c,    res_5_d,    res_5_e,   res_5_h,  res_5_l,  res_5_dhl, res_5_a,
    /*Bx*/ res_6_b,  res_6_c,  res_6_d,   res_6_e,  res_6_h,  res_6_l,   res_6_dhl,  res_6_a,  res_7_b,    res_7_c,    res_7_d,    res_7_e,   res_7_h,  res_7_l,  res_7_dhl, res_7_a,
    /*Cx*/ set_0_b,  set_0_c,  set_0_d,   set_0_e,  set_0_h,  set_0_l,   set_0_dhl,  set_0_a,  set_1_b,    set_1_c,    set_1_d,    set_1_e,   set_1_h,  set_1_l,  set_1_dhl, set_1_a,
    /*Dx*/ set_2_b,  set_2_c,  set_2_d,   set_2_e,  set_2_h,  set_2_l,   set_2_dhl,  set_2_a,  set_3_b,    set_3_c,    set_3_d,    set_3_e,   set_3_h,  set_3_l,  set_3_dhl, set_3_a,
    /*Ex*/ set_4_b,  set_4_c,  set_4_d,   set_4_e,  set_4_h,  set_4_l,   set_4_dhl,  set_4_a,  set_5_b,    set_5_c,    set_5_d,    set_5_e,   set_5_h,  set_5_l,  set_5_dhl, set_5_a,
    /*Fx*/ set_6_b,  set_6_c,  set_6_d,   set_6_e,  set_6_h,  set_6_l,   set_6_dhl,  set_6_a,  set_7_b,    set_7_c,    set_7_d,    set_7_e,   set_7_h,  set_7_l,  set_7_dhl, set_7_a
    };

    static_assert(std::size(cb_map) == 0x100, "lost some cb opcodes");

};

void Cpu::mode_cb() {
    auto opcode = imm8();
    std::invoke(CpuInstrTable::cb_map[opcode], this);
}

void Cpu::step() {
    _cycle_count = 0;

    if (_halted) {
        if (pending_interrupts()) {
            _halted = false;
        } else {
            _cycle_count += _bus->advance();
            return;
        }
    }

    if (_stopped) {
        if (_bus->read_byte(int_flag_addr) & int_joypad_bit) {
            _stopped = false;
        } else {
            _cycle_count += _bus->advance();
            return;
        }
    }

    if(check_interrupts()) {
        return;
    }

    auto opcode = imm8();
    std::invoke(CpuInstrTable::base[opcode], this);

    if (_ime_delay > 0) {
        _ime_delay--;
        if (_ime_delay == 0) {
            _interrupts_enable = true;
        }
    }

    _just_wrote_if = false;
}

} // namespace bdmg
