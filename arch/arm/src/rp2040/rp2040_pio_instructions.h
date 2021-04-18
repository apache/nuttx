/****************************************************************************
 * arch/arm/src/rp2040/rp2040_pio_instructions.h
 *
 * Based upon the software originally developed by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_PIO_INSTRUCTIONS_H
#define __ARCH_ARM_SRC_RP2040_RP2040_PIO_INSTRUCTIONS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include "rp2040_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_PIO_INSTRUCTIONS,
 * Enable/disable assertions in the PIO instructions, type=bool, default=0,
 * group=hardware_pio
 */

#ifndef PARAM_ASSERTIONS_ENABLED_PIO_INSTRUCTIONS
#define PARAM_ASSERTIONS_ENABLED_PIO_INSTRUCTIONS 0
#endif

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum pio_instr_bits
  {
    pio_instr_bits_jmp = 0x0000,
    pio_instr_bits_wait = 0x2000,
    pio_instr_bits_in = 0x4000,
    pio_instr_bits_out = 0x6000,
    pio_instr_bits_push = 0x8000,
    pio_instr_bits_pull = 0x8080,
    pio_instr_bits_mov = 0xa000,
    pio_instr_bits_irq = 0xc000,
    pio_instr_bits_set = 0xe000,
  };

#ifndef NDEBUG
#define _PIO_INVALID_IN_SRC    0x08u
#define _PIO_INVALID_OUT_DEST 0x10u
#define _PIO_INVALID_SET_DEST 0x20u
#define _PIO_INVALID_MOV_SRC  0x40u
#define _PIO_INVALID_MOV_DEST 0x80u
#else
#define _PIO_INVALID_IN_SRC    0u
#define _PIO_INVALID_OUT_DEST 0u
#define _PIO_INVALID_SET_DEST 0u
#define _PIO_INVALID_MOV_SRC  0u
#define _PIO_INVALID_MOV_DEST 0u
#endif

enum pio_src_dest
  {
    pio_pins = 0u,
    pio_x = 1u,
    pio_y = 2u,
    pio_null =     3u | _PIO_INVALID_SET_DEST | _PIO_INVALID_MOV_DEST,
    pio_pindirs =  4u | _PIO_INVALID_IN_SRC | _PIO_INVALID_MOV_SRC |
                        _PIO_INVALID_MOV_DEST,
    pio_exec_mov = 4u | _PIO_INVALID_IN_SRC | _PIO_INVALID_OUT_DEST |
                        _PIO_INVALID_SET_DEST | _PIO_INVALID_MOV_SRC,
    pio_status =   5u | _PIO_INVALID_IN_SRC | _PIO_INVALID_OUT_DEST |
                        _PIO_INVALID_SET_DEST | _PIO_INVALID_MOV_DEST,
    pio_pc =       5u | _PIO_INVALID_IN_SRC | _PIO_INVALID_SET_DEST |
                        _PIO_INVALID_MOV_SRC,
    pio_isr =      6u | _PIO_INVALID_SET_DEST,
    pio_osr =      7u | _PIO_INVALID_OUT_DEST | _PIO_INVALID_SET_DEST,
    pio_exec_out = 7u | _PIO_INVALID_IN_SRC | _PIO_INVALID_SET_DEST |
                        _PIO_INVALID_MOV_SRC | _PIO_INVALID_MOV_DEST,
  };

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

inline static uint32_t _pio_major_instr_bits(uint32_t instr)
{
  return instr & 0xe000u;
}

inline static uint32_t _pio_encode_instr_and_args(
                                              enum pio_instr_bits instr_bits,
                                              uint32_t arg1, uint32_t arg2)
{
  valid_params_if(PIO_INSTRUCTIONS, arg1 <= 0x7);
#if PARAM_ASSERTIONS_ENABLED_PIO_INSTRUCTIONS
  uint32_t major = _pio_major_instr_bits(instr_bits);
  if (major == pio_instr_bits_in || major == pio_instr_bits_out)
    {
      assert(arg2 && arg2 <= 32);
    }
  else
    {
      assert(arg2 <= 31);
    }
#endif
  return instr_bits | (arg1 << 5u) | (arg2 & 0x1fu);
}

inline static uint32_t _pio_encode_instr_and_src_dest(
                                            enum pio_instr_bits instr_bits,
                                            enum pio_src_dest dest,
                                            uint32_t value)
{
  return _pio_encode_instr_and_args(instr_bits, dest & 7u, value);
}

inline static uint32_t pio_encode_delay(uint32_t cycles)
{
  valid_params_if(PIO_INSTRUCTIONS, cycles <= 0x1f);
  return cycles << 8u;
}

inline static uint32_t pio_encode_sideset(uint32_t sideset_bit_count,
                                          uint32_t value)
{
  valid_params_if(PIO_INSTRUCTIONS,
                  sideset_bit_count >= 1 && sideset_bit_count <= 5);
  valid_params_if(PIO_INSTRUCTIONS, value <= (0x1fu >> sideset_bit_count));
  return value << (13u - sideset_bit_count);
}

inline static uint32_t pio_encode_sideset_opt(uint32_t sideset_bit_count,
                                              uint32_t value)
{
  valid_params_if(PIO_INSTRUCTIONS,
                  sideset_bit_count >= 2 && sideset_bit_count <= 5);
  valid_params_if(PIO_INSTRUCTIONS, value <= (0x1fu >> sideset_bit_count));
  return 0x1000u | value << (12u - sideset_bit_count);
}

inline static uint32_t pio_encode_jmp(uint32_t addr)
{
  return _pio_encode_instr_and_args(pio_instr_bits_jmp, 0, addr);
}

inline static uint32_t _pio_encode_irq(bool relative, uint32_t irq)
{
  valid_params_if(PIO_INSTRUCTIONS, irq <= 7);
  return (relative ? 0x10u : 0x0u) | irq;
}

inline static uint32_t pio_encode_wait_gpio(bool polarity, uint32_t pin)
{
  return _pio_encode_instr_and_args(pio_instr_bits_wait,
                                    0u | (polarity ? 4u : 0u), pin);
}

inline static uint32_t pio_encode_wait_pin(bool polarity, uint32_t pin)
{
  return _pio_encode_instr_and_args(pio_instr_bits_wait,
                                    1u | (polarity ? 4u : 0u), pin);
}

inline static uint32_t pio_encode_wait_irq(bool polarity, bool relative,
                                       uint32_t irq)
{
  valid_params_if(PIO_INSTRUCTIONS, irq <= 7);
  return _pio_encode_instr_and_args(pio_instr_bits_wait,
                                    2u | (polarity ? 4u : 0u),
                                    _pio_encode_irq(relative, irq));
}

inline static uint32_t pio_encode_in(enum pio_src_dest src, uint32_t value)
{
  valid_params_if(PIO_INSTRUCTIONS, !(src & _PIO_INVALID_IN_SRC));
  return _pio_encode_instr_and_src_dest(pio_instr_bits_in, src, value);
}

inline static uint32_t pio_encode_out(enum pio_src_dest dest, uint32_t value)
{
  valid_params_if(PIO_INSTRUCTIONS, !(dest & _PIO_INVALID_OUT_DEST));
  return _pio_encode_instr_and_src_dest(pio_instr_bits_out, dest, value);
}

inline static uint32_t pio_encode_push(bool if_full, bool block)
{
  return _pio_encode_instr_and_args(pio_instr_bits_push,
                                    (if_full ? 2u : 0u) | (block ? 1u : 0u),
                                    0);
}

inline static uint32_t pio_encode_pull(bool if_empty, bool block)
{
  return _pio_encode_instr_and_args(pio_instr_bits_pull,
                                    (if_empty ? 2u : 0u) | (block ? 1u : 0u),
                                    0);
}

inline static uint32_t pio_encode_mov(enum pio_src_dest dest,
                                  enum pio_src_dest src)
{
  valid_params_if(PIO_INSTRUCTIONS, !(dest & _PIO_INVALID_MOV_DEST));
  valid_params_if(PIO_INSTRUCTIONS, !(src & _PIO_INVALID_MOV_SRC));
  return _pio_encode_instr_and_src_dest(pio_instr_bits_mov, dest,
                                        src & 7u);
}

inline static uint32_t pio_encode_mov_not(enum pio_src_dest dest,
                                      enum pio_src_dest src)
{
  valid_params_if(PIO_INSTRUCTIONS, !(dest & _PIO_INVALID_MOV_DEST));
  valid_params_if(PIO_INSTRUCTIONS, !(src & _PIO_INVALID_MOV_SRC));
  return _pio_encode_instr_and_src_dest(pio_instr_bits_mov, dest,
                                        (1u << 3u) | (src & 7u));
}

inline static uint32_t pio_encode_mov_reverse(enum pio_src_dest dest,
                                          enum pio_src_dest src)
{
  valid_params_if(PIO_INSTRUCTIONS, !(dest & _PIO_INVALID_MOV_DEST));
  valid_params_if(PIO_INSTRUCTIONS, !(src & _PIO_INVALID_MOV_SRC));
  return _pio_encode_instr_and_src_dest(pio_instr_bits_mov, dest,
                                        (2u << 3u) | (src & 7u));
}

inline static uint32_t pio_encode_irq_set(bool relative, uint32_t irq)
{
  return _pio_encode_instr_and_args(pio_instr_bits_irq, 0,
                                    _pio_encode_irq(relative, irq));
}

inline static uint32_t pio_encode_irq_clear(bool relative, uint32_t irq)
{
  return _pio_encode_instr_and_args(pio_instr_bits_irq, 2,
                                    _pio_encode_irq(relative, irq));
}

inline static uint32_t pio_encode_set(enum pio_src_dest dest, uint32_t value)
{
  valid_params_if(PIO_INSTRUCTIONS, !(dest & _PIO_INVALID_SET_DEST));
  return _pio_encode_instr_and_src_dest(pio_instr_bits_set, dest, value);
}

inline static uint32_t pio_encode_nop(void)
{
  return pio_encode_mov(pio_y, pio_y);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP2040_RP2040_PIO_INSTRUNCTIONS_H */
