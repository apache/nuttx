/****************************************************************************
 * arch/arm/src/rp2040/rp2040_pio.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spinlock.h>

#include <arch/board/board.h>

#include "hardware/rp2040_pio.h"
#include "rp2040_pio.h"
#include "rp2040_pio_instructions.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SMP
#define hw_claim_lock()         spin_lock_irqsave(&pio_lock)
#define hw_claim_unlock(save)   spin_unlock_irqrestore(&pio_lock, save)
#else
#define hw_claim_lock()         spin_lock_irqsave(NULL)
#define hw_claim_unlock(save)   spin_unlock_irqrestore(NULL, save)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SMP
static spinlock_t pio_lock;
#endif

static uint8_t claimed;

static uint32_t _used_instruction_space[2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void hw_claim_or_assert(uint8_t *bits, uint32_t bit_index,
                               const char *message)
{
  uint32_t save = hw_claim_lock();
  if (bits[bit_index >> 3u] & (1u << (bit_index & 7u)))
    {
      DEBUGPANIC();
    }
  else
    {
      bits[bit_index >> 3u] |= (uint8_t)(1u << (bit_index & 7u));
    }

  hw_claim_unlock(save);
}

static int hw_claim_unused_from_range(uint8_t *bits, bool required,
                                      uint32_t bit_lsb, uint32_t bit_msb,
                                      const char *message)
{
  /* don't bother check lsb / msb order as if wrong, then it'll fail anyway */

  uint32_t save = hw_claim_lock();
  int found_bit = -1;
  for (uint32_t bit = bit_lsb; bit <= bit_msb; bit++)
    {
      if (!(bits[bit >> 3u] & (1u << (bit & 7u))))
        {
          bits[bit >> 3u] |= (uint8_t)(1u << (bit & 7u));
          found_bit = (int)bit;
          break;
        }
    }

  hw_claim_unlock(save);
  if (found_bit < 0 && required)
    {
      DEBUGPANIC();
    }

  return found_bit;
}

static void hw_claim_clear(uint8_t *bits, uint32_t bit_index)
{
  uint32_t save = hw_claim_lock();
  assert(bits[bit_index >> 3u] & (1u << (bit_index & 7u)));
  bits[bit_index >> 3u] &= (uint8_t) ~(1u << (bit_index & 7u));
  hw_claim_unlock(save);
}

static int _pio_find_offset_for_program(uint32_t pio,
                                        const rp2040_pio_program_t *program)
{
  assert(program->length < PIO_INSTRUCTION_COUNT);
  uint32_t used_mask = _used_instruction_space[rp2040_pio_get_index(pio)];
  uint32_t program_mask = (1u << program->length) - 1;

  if (program->origin >= 0)
    {
      if (program->origin > 32 - program->length)
        {
          return -1;
        }

      return used_mask & (program_mask << program->origin) ?
              -1 : program->origin;
    }
  else
    {
      /* work down from the top always */

      for (int i = 32 - program->length; i >= 0; i--)
        {
          if (!(used_mask & (program_mask << (uint32_t) i)))
            {
              return i;
            }
        }

      return -1;
    }
}

static bool _pio_can_add_program_at_offset(uint32_t pio,
                                        const rp2040_pio_program_t *program,
                                        uint32_t offset)
{
  valid_params_if(PIO, offset < PIO_INSTRUCTION_COUNT);
  valid_params_if(PIO, offset + program->length <= PIO_INSTRUCTION_COUNT);
  if (program->origin >= 0 && (uint32_t)program->origin != offset)
    {
      return false;
    }

  uint32_t used_mask = _used_instruction_space[rp2040_pio_get_index(pio)];
  uint32_t program_mask = (1u << program->length) - 1;
  return !(used_mask & (program_mask << offset));
}

static void _pio_add_program_at_offset(uint32_t pio,
                                       const rp2040_pio_program_t *program,
                                       uint32_t offset)
{
  if (!_pio_can_add_program_at_offset(pio, program, offset))
    {
      DEBUGPANIC(); /* "No program space" */
    }

  for (uint32_t i = 0; i < program->length; ++i)
    {
      uint16_t instr = program->instructions[i];
      putreg32(pio_instr_bits_jmp != _pio_major_instr_bits(instr) ?
               instr : instr + offset,
               RP2040_PIO_INSTR_MEM(pio, offset + i));
    }

  uint32_t program_mask = (1u << program->length) - 1;
  _used_instruction_space[rp2040_pio_get_index(pio)] |=
    program_mask << offset;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rp2040_pio_sm_claim(uint32_t pio, uint32_t sm)
{
  check_sm_param(sm);
  uint32_t which = rp2040_pio_get_index(pio);

  if (which)
    {
      hw_claim_or_assert(&claimed, RP2040_PIO_SM_NUM + sm,
                         "PIO 1 SM (%d - 4) already claimed");
    }
  else
    {
      hw_claim_or_assert(&claimed, sm,
                         "PIO 0 SM %d already claimed");
    }
}

void rp2040_pio_claim_sm_mask(uint32_t pio, uint32_t sm_mask)
{
  for (uint32_t i = 0; sm_mask; i++, sm_mask >>= 1u)
    {
      if (sm_mask & 1u)
        {
           rp2040_pio_sm_claim(pio, i);
        }
    }
}

void rp2040_pio_sm_unclaim(uint32_t pio, uint32_t sm)
{
  check_sm_param(sm);
  uint32_t which = rp2040_pio_get_index(pio);
  hw_claim_clear(&claimed, which * RP2040_PIO_SM_NUM + sm);
}

int rp2040_pio_claim_unused_sm(uint32_t pio, bool required)
{
  /* PIO index is 0 or 1. */

  uint32_t which = rp2040_pio_get_index(pio);
  uint32_t base = which * RP2040_PIO_SM_NUM;
  int index = hw_claim_unused_from_range((uint8_t *)&claimed, required, base,
                                      base + RP2040_PIO_SM_NUM - 1,
                                      "No PIO state machines are available");
  return index >= (int)base ? index - (int)base : -1;
}

bool rp2040_pio_can_add_program(uint32_t pio,
                                const rp2040_pio_program_t *program)
{
  uint32_t save = hw_claim_lock();
  bool rc =  -1 != _pio_find_offset_for_program(pio, program);
  hw_claim_unlock(save);
  return rc;
}

bool rp2040_pio_can_add_program_at_offset(uint32_t pio,
                                        const rp2040_pio_program_t *program,
                                        uint32_t offset)
{
  uint32_t save = hw_claim_lock();
  bool rc = _pio_can_add_program_at_offset(pio, program, offset);
  hw_claim_unlock(save);
  return rc;
}

/* these assert if unable */

uint32_t rp2040_pio_add_program(uint32_t pio,
                                const rp2040_pio_program_t *program)
{
  uint32_t save = hw_claim_lock();
  int offset = _pio_find_offset_for_program(pio, program);
  if (offset < 0)
    {
      DEBUGPANIC(); /* "No program space" */
    }

  _pio_add_program_at_offset(pio, program, (uint32_t)offset);
  hw_claim_unlock(save);
  return (uint32_t)offset;
}

void rp2040_pio_add_program_at_offset(uint32_t pio,
                                      const rp2040_pio_program_t *program,
                                      uint32_t offset)
{
  uint32_t save = hw_claim_lock();
  _pio_add_program_at_offset(pio, program, offset);
  hw_claim_unlock(save);
}

void rp2040_pio_remove_program(uint32_t pio,
                               const rp2040_pio_program_t *program,
                               uint32_t loaded_offset)
{
  uint32_t program_mask = (1u << program->length) - 1;
  program_mask <<= loaded_offset;
  uint32_t save = hw_claim_lock();
  assert(program_mask ==
         (_used_instruction_space[rp2040_pio_get_index(pio)] &
            program_mask));
  _used_instruction_space[rp2040_pio_get_index(pio)] &= ~program_mask;
  hw_claim_unlock(save);
}

void rp2040_pio_clear_instruction_memory(uint32_t pio)
{
  uint32_t save = hw_claim_lock();
  _used_instruction_space[rp2040_pio_get_index(pio)] = 0;
  for (uint32_t i = 0; i < PIO_INSTRUCTION_COUNT; i++)
    {
      putreg32(pio_encode_jmp(i), RP2040_PIO_INSTR_MEM(pio, i));
    }

  hw_claim_unlock(save);
}

/* Set the value of all PIO pins. This is done by forcibly executing
 * instructions on a "victim" state machine, sm. Ideally you should choose
 * one which is not currently running a program. This is intended for
 * one-time setup of initial pin states.
 */

void rp2040_pio_sm_set_pins(uint32_t pio, uint32_t sm, uint32_t pins)
{
  check_pio_param(pio);
  check_sm_param(sm);
  uint32_t pinctrl_saved = getreg32(RP2040_PIO_SM_PINCTRL(pio, sm));
  uint32_t remaining = 32;
  uint32_t base = 0;

  while (remaining)
    {
      uint32_t decrement = remaining > 5 ? 5 : remaining;
      putreg32((decrement << RP2040_PIO_SM_PINCTRL_SET_COUNT_SHIFT) |
               (base << RP2040_PIO_SM_PINCTRL_SET_BASE_SHIFT),
               RP2040_PIO_SM_PINCTRL(pio, sm));
      rp2040_pio_sm_exec(pio, sm,
                         pio_encode_set(pio_pins, pins & 0x1fu));
      remaining -= decrement;
      base += decrement;
      pins >>= 5;
    }

  putreg32(pinctrl_saved, RP2040_PIO_SM_PINCTRL(pio, sm));
}

void rp2040_pio_sm_set_pins_with_mask(uint32_t pio, uint32_t sm,
                                    uint32_t pinvals, uint32_t pin_mask)
{
  check_pio_param(pio);
  check_sm_param(sm);
  uint32_t pinctrl_saved = getreg32(RP2040_PIO_SM_PINCTRL(pio, sm));

  while (pin_mask)
    {
      uint32_t base = (uint32_t)__builtin_ctz(pin_mask);
      putreg32((1u << RP2040_PIO_SM_PINCTRL_SET_COUNT_SHIFT) |
               (base << RP2040_PIO_SM_PINCTRL_SET_BASE_SHIFT),
               RP2040_PIO_SM_PINCTRL(pio, sm));
      rp2040_pio_sm_exec(pio, sm,
                         pio_encode_set(pio_pins,
                                        (pinvals >> base) & 0x1u));
      pin_mask &= pin_mask - 1;
    }

  putreg32(pinctrl_saved, RP2040_PIO_SM_PINCTRL(pio, sm));
}

void rp2040_pio_sm_set_pindirs_with_mask(uint32_t pio, uint32_t sm,
                                         uint32_t pindirs,
                                         uint32_t pin_mask)
{
  check_pio_param(pio);
  check_sm_param(sm);
  uint32_t pinctrl_saved = getreg32(RP2040_PIO_SM_PINCTRL(pio, sm));

  while (pin_mask)
    {
      uint32_t base = (uint32_t)__builtin_ctz(pin_mask);
      putreg32((1u << RP2040_PIO_SM_PINCTRL_SET_COUNT_SHIFT) |
               (base << RP2040_PIO_SM_PINCTRL_SET_BASE_SHIFT),
               RP2040_PIO_SM_PINCTRL(pio, sm));
      rp2040_pio_sm_exec(pio, sm,
                         pio_encode_set(pio_pindirs,
                                        (pindirs >> base) & 0x1u));
      pin_mask &= pin_mask - 1;
    }

  putreg32(pinctrl_saved, RP2040_PIO_SM_PINCTRL(pio, sm));
}

void rp2040_pio_sm_set_consecutive_pindirs(uint32_t pio, uint32_t sm,
                                           uint32_t pin,
                                           uint32_t count, bool is_out)
{
  check_pio_param(pio);
  check_sm_param(sm);
  valid_params_if(PIO, pin < 32u);

  uint32_t pinctrl_saved = getreg32(RP2040_PIO_SM_PINCTRL(pio, sm));
  uint32_t pindir_val = is_out ? 0x1f : 0;

  while (count > 5)
    {
      putreg32((5u << RP2040_PIO_SM_PINCTRL_SET_COUNT_SHIFT) |
               (pin << RP2040_PIO_SM_PINCTRL_SET_BASE_SHIFT),
               RP2040_PIO_SM_PINCTRL(pio, sm));
      rp2040_pio_sm_exec(pio, sm,
                         pio_encode_set(pio_pindirs, pindir_val));
      count -= 5;
      pin = (pin + 5) & 0x1f;
    }

  putreg32((count << RP2040_PIO_SM_PINCTRL_SET_COUNT_SHIFT) |
           (pin << RP2040_PIO_SM_PINCTRL_SET_BASE_SHIFT),
           RP2040_PIO_SM_PINCTRL(pio, sm));
  rp2040_pio_sm_exec(pio, sm,
                     pio_encode_set(pio_pindirs, pindir_val));
  putreg32(pinctrl_saved, RP2040_PIO_SM_PINCTRL(pio, sm));
}

void rp2040_pio_sm_init(uint32_t pio, uint32_t sm, uint32_t initial_pc,
                        const rp2040_pio_sm_config *config)
{
  valid_params_if(PIO, initial_pc < PIO_INSTRUCTION_COUNT);

  /* Halt the machine, set some sensible defaults */

  rp2040_pio_sm_set_enabled(pio, sm, false);

  if (config)
    {
      rp2040_pio_sm_set_config(pio, sm, config);
    }
  else
    {
      rp2040_pio_sm_config c = rp2040_pio_get_default_sm_config();
      rp2040_pio_sm_set_config(pio, sm, &c);
    }

  rp2040_pio_sm_clear_fifos(pio, sm);

  /* Clear FIFO debug flags */

  const uint32_t fdebug_sm_mask = (1u << RP2040_PIO_FDEBUG_TXOVER_SHIFT) |
                                  (1u << RP2040_PIO_FDEBUG_RXUNDER_SHIFT) |
                                  (1u << RP2040_PIO_FDEBUG_TXSTALL_SHIFT) |
                                  (1u << RP2040_PIO_FDEBUG_RXSTALL_SHIFT);
  putreg32(fdebug_sm_mask << sm, RP2040_PIO_FDEBUG(pio));

  /* Finally, clear some internal SM state */

  rp2040_pio_sm_restart(pio, sm);
  rp2040_pio_sm_clkdiv_restart(pio, sm);
  rp2040_pio_sm_exec(pio, sm, pio_encode_jmp(initial_pc));
}

void rp2040_pio_sm_drain_tx_fifo(uint32_t pio, uint32_t sm)
{
  uint32_t instr = (getreg32(RP2040_PIO_SM_SHIFTCTRL(pio, sm)) &
                    RP2040_PIO_SM_SHIFTCTRL_AUTOPULL) ?
                   pio_encode_out(pio_null, 32) :
                   pio_encode_pull(false, false);

  while (!rp2040_pio_sm_is_tx_fifo_empty(pio, sm))
    {
      rp2040_pio_sm_exec(pio, sm, instr);
    }
}
