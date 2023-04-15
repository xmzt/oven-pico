#ifndef UTIL_H
#define UTIL_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

inline static void util_alarm_set(unsigned alarm_id, unsigned irq_id, void (*irq)(void), uint32_t time_lo) {
	hw_set_bits(&timer_hw->inte, 1u << alarm_id); // enable interrupt for alarm
	irq_set_exclusive_handler(irq_id, irq);
	irq_set_enabled(irq_id, true);
	timer_hw->alarm[alarm_id] = time_lo;
}

inline static void util_alarm_inc(unsigned alarm_id, uint32_t inc_lo) {
	//hw_set_bits(&timer_hw->inte, 1u << alarm_id); // enable interrupt for alarm
	//irq_set_exclusive_handler(irq_id, irq);
	//irq_set_enabled(irq_id, true);
	timer_hw->alarm[alarm_id] += inc_lo;
}

inline static uint util_pio_program_load(PIO pio, const pio_program_t *program, uint offset) {
	offset -= program->length;
	for (uint i = 0; i < program->length; ++i) {
        uint16_t instr = program->instructions[i];
        pio->instr_mem[offset + i] = pio_instr_bits_jmp != _pio_major_instr_bits(instr) ? instr : instr + offset;
    }
	return offset;
}

inline static uint util_pio_program_clear(PIO pio, uint offset, uint clearN) {
	for (uint offsetE = offset + clearN; offset < offsetE; offset++) {
        pio->instr_mem[offset] = pio_encode_jmp(offset);
	}
	return offset;
}

#endif
