#pragma once

#include "common.hh"
#include "bus.hh"
#include "boot-rom.hh"

namespace bdmg {

union RegisterFile {
    enum Ids {
        raf = 0,
        rbc = 1,
        rde = 2,
        rhl = 3,
        rsp = 4,
        rpc = 5,
        total
    };

    u16 list[total];
    struct {
        u16 af;
        u16 bc;
        u16 de;
        u16 hl;
        u16 sp;
        u16 pc;
    };
    struct {
        u8 f;
        u8 a;
        u8 c;
        u8 b;
        u8 e;
        u8 d;
        u8 l;
        u8 h;
    };

};

using RF = RegisterFile;


class Cpu {
    friend struct CpuInstrTable;
public:
    Cpu() = default;
    Cpu(const Cpu&) = default;
    Cpu& operator=(const Cpu&) = default;
    Cpu(const SPBus&, const SPBootRom&);

    std::string print_state();

    void step();

    RegisterFile& regs() {
        return _regs;
    }

    const RegisterFile& regs() const {
        return _regs;
    }

private:

    using StepFn = void (Cpu::*)();

    void nop();

    template<u8 opcode> void hcf();

    void ld_hi_a();
    void ld_a_hi();

    template<auto src>
    void ld_a_drr();

    template<auto dst>
    void ldr_i16();

    void ld_hldec();
    void ld_hlinc();
    void ld_a_hlinc();
    void ld_a_hldec();
    void ldhic_a();
    void lda_hic();
    void ld_di16_a();
    void ld_a_di16();
    
    void jp();
    void jp_hl();
    void jp_nc();
    void jp_nz();
    void jp_z();
    void jp_c();

    void jr();
    void ret();

    template<auto cond>
    void jr_cc();

    template<auto cond>
    void ret_cc();

    void call_r16();
    void call_nz();
    void call_nc();
    void call_z();
    void call_c();

    void ld_hl_sp_s8();
    void ld_sp_hl();

    template<u16 addr>
    void rst();

    template<auto src>
    void push_r16();

    template<auto dst>
    void pop_r16();

    void adc_a_i8();
    void sbc_a_i8();
    void xor_a_i8();
    void cmp_a_i8();

    void add_a_i8();
    void sub_a_i8();
    void and_a_i8();
    void or_a_i8();

    void ld_a16_sp();

    void di();
    void ei();
    void reti();

    void rlca();
    void rrca();
    void rla();
    void rra();

    void cpl();
    void ccf();
    void scf();

    void inc_dhl();
    void dec_dhl();

    void stop();
    void halt();

    void daa();

    void mode_cb();

    void ld_dhl_i8();

    template<auto arg>
    void cb_rlc();
    void cb_rlc_m();

    template<auto arg>
    void cb_rrc();
    void cb_rrc_m();

    template<auto arg>
    void cb_rl();
    void cb_rl_m();

    template<auto arg>
    void cb_rr();
    void cb_rr_m();

    template<auto arg>
    void cb_sla();
    void cb_sla_m();

    template<auto arg>
    void cb_sra();
    void cb_sra_m();

    template<auto arg>
    void cb_swap();
    void cb_swap_m();

    template<auto arg>
    void cb_srl();
    void cb_srl_m();

    template<auto arg, u8 bit>
    void cb_test();
    template<u8 bit>
    void cb_test_m();

    template<auto arg, u8 bit>
    void cb_set();
    template<u8 bit>
    void cb_set_m();

    template<auto arg, u8 bit>
    void cb_reset();
    template<u8 bit>
    void cb_reset_m();

    template<auto dst>
    void ld_dst_i8();

    template<auto dst, auto src>
    void ld_dst_src();

    template<auto src>
    void ld_dhl_src();

    template<auto dst>
    void ld_dst_dhl();

    template<auto dst>
    void ld_dst_a();

    template<auto dst>
    void inc_dst8();

    template<auto dst>
    void dec_dst8();

    template<auto src>
    void add_hl_src();

    void add_sp_s8();

    template<auto src>
    void add_a_src();

    template<auto src>
    void adc_a_src();

    template<auto src>
    void sub_a_src();

    template<auto src>
    void sbc_a_src();

    template<auto src>
    void and_dst_src();

    template<auto src>
    void xor_a_src();

    template<auto src>
    void or_a_src();

    template<auto src>
    void cp_a_src();

    void add_a_dhl();
    void adc_a_dhl();
    void sub_a_dhl();
    void sbc_a_dhl();
    void and_a_dhl();
    void xor_a_dhl();
    void or_a_dhl();
    void cp_a_dhl();

    template<auto dst>
    void inc_dst16();

    template<auto dst>
    void dec_dst16();

    u8 read_bus_byte(u16 addr);
    void write_bus_byte(u16 addr, u8 data);

    u8 read_boot_byte(u16 addr);
    void write_boot_byte(u16 addr, u8 data);

    u8 read_byte(u16 addr);
    void write_byte(u16 addr, u8 data);

    s8 signed_read_byte(u16 addr);

    u16 read_word(u16 addr);

    void push_sp_word(u16 data);

    u8 imm8();
    u16 imm16();

    bool pending_interrupts() const noexcept;

    bool check_interrupts();

    SPBus _bus;
    SPBootRom _boot_rom;

    RegisterFile _regs {};
    u64 _cycle_count = 0;
    bool _interrupts_enable = false;
    int _ime_delay = 0; 
    bool _stopped = false;
    bool _halted = false;
    bool _just_wrote_if = false;

    decltype(&Cpu::read_boot_byte) _read_byte = &Cpu::read_boot_byte;
    decltype(&Cpu::write_boot_byte) _write_byte = &Cpu::write_boot_byte;
};

} // namespace bdmg
