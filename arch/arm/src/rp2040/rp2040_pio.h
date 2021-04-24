/****************************************************************************
 * arch/arm/src/rp2040/rp2040_pio.h
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

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_PIO_H
#define __ARCH_ARM_SRC_RP2040_RP2040_PIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "rp2040_gpio.h"
#include "rp2040_dmac.h"
#include "hardware/rp2040_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP2040_PIO_NUM        2     /* Number of PIOs */
#define RP2040_PIO_SM_NUM     4     /* Number of state machines per PIO */

#define PIO_INSTRUCTION_COUNT   32u

#define valid_params_if(x, test)    DEBUGASSERT(test)

#define check_sm_param(sm)   valid_params_if(PIO, sm < RP2040_PIO_SM_NUM)
#define check_pio_param(pio) valid_params_if(PIO, pio < RP2040_PIO_NUM)

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

/* FIFO join states */

enum rp2040_pio_fifo_join
  {
    RP2040_PIO_FIFO_JOIN_NONE = 0,
    RP2040_PIO_FIFO_JOIN_TX = 1,
    RP2040_PIO_FIFO_JOIN_RX = 2,
  };

/* MOV status types */

enum rp2040_pio_mov_status_type
  {
    RP2040_STATUS_TX_LESSTHAN = 0,
    RP2040_STATUS_RX_LESSTHAN = 1
  };

/* PIO Configuration structure */

typedef struct
  {
    uint32_t clkdiv;
    uint32_t execctrl;
    uint32_t shiftctrl;
    uint32_t pinctrl;
  }
rp2040_pio_sm_config;

typedef struct rp2040_pio_program
  {
    const uint16_t *instructions;
    uint8_t length;
    int8_t origin; /* required instruction memory origin or -1 */
  }
rp2040_pio_program_t;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_sm_config_set_out_pins
 *
 * Description:
 *   Set the 'out' pins in a state machine configuration
 *   Can overlap with the 'in', 'set' and 'sideset' pins
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   out_base - 0-31 First pin to set as output
 *   out_count - 0-32 Number of pins to set.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_out_pins(rp2040_pio_sm_config *c,
                                                 uint32_t out_base,
                                                 uint32_t out_count)
{
  valid_params_if(PIO, out_base < 32);
  valid_params_if(PIO, out_count <= 32);
  c->pinctrl = (c->pinctrl & ~(RP2040_PIO_SM_PINCTRL_OUT_BASE_MASK |
                               RP2040_PIO_SM_PINCTRL_OUT_COUNT_MASK)) |
               (out_base << RP2040_PIO_SM_PINCTRL_OUT_BASE_SHIFT) |
               (out_count << RP2040_PIO_SM_PINCTRL_OUT_COUNT_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_set_pins
 *
 * Description:
 *   Set the 'set' pins in a state machine configuration
 *   Can overlap with the 'in', 'out' and 'sideset' pins
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   set_base - 0-31 First pin to set as
 *   set_count - 0-5 Number of pins to set.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_set_pins(rp2040_pio_sm_config *c,
                                                 uint32_t set_base,
                                                 uint32_t set_count)
{
  valid_params_if(PIO, set_base < 32);
  valid_params_if(PIO, set_count <= 5);
  c->pinctrl = (c->pinctrl & ~(RP2040_PIO_SM_PINCTRL_SET_BASE_MASK |
                               RP2040_PIO_SM_PINCTRL_SET_COUNT_MASK)) |
               (set_base << RP2040_PIO_SM_PINCTRL_SET_BASE_SHIFT) |
               (set_count << RP2040_PIO_SM_PINCTRL_SET_COUNT_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_in_pins
 *
 * Description:
 *   Set the 'in' pins in a state machine configuration
 *   Can overlap with the 'out', ''set' and 'sideset' pins
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   in_base - 0-31 First pin to use as input
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_in_pins(rp2040_pio_sm_config *c,
                                                uint32_t in_base)
{
  valid_params_if(PIO, in_base < 32);
  c->pinctrl = (c->pinctrl & ~RP2040_PIO_SM_PINCTRL_IN_BASE_MASK) |
               (in_base << RP2040_PIO_SM_PINCTRL_IN_BASE_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_sideset_pins
 *
 * Description:
 *   Set the 'sideset' pins in a state machine configuration
 *   Can overlap with the 'in', 'out' and 'set' pins
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   sideset_base - 0-31 base pin for 'side set'
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_sideset_pins(rp2040_pio_sm_config *c,
                                                     uint32_t sideset_base)
{
  valid_params_if(PIO, sideset_base < 32);
  c->pinctrl = (c->pinctrl & ~RP2040_PIO_SM_PINCTRL_SIDESET_BASE_MASK) |
               (sideset_base << RP2040_PIO_SM_PINCTRL_SIDESET_BASE_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_sideset
 *
 * Description:
 *   Set the 'sideset' options in a state machine configuration
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   bit_count - Number of bits to steal from delay field in the
 *               instruction for use of side set (max 5)
 *   optional - True if the topmost side set bit is used as a flag for
 *              whether to apply side set on that instruction
 *   pindirs - True if the side set affects pin directions rather than values
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_sideset(rp2040_pio_sm_config *c,
                                                uint32_t bit_count,
                                                bool optional,
                                                bool pindirs)
{
  valid_params_if(PIO, bit_count <= 5);
  valid_params_if(PIO, !optional || bit_count >= 1);
  c->pinctrl = (c->pinctrl & ~RP2040_PIO_SM_PINCTRL_SIDESET_COUNT_MASK) |
               (bit_count << RP2040_PIO_SM_PINCTRL_SIDESET_COUNT_SHIFT);

  c->execctrl = (c->execctrl & ~(RP2040_PIO_SM_EXECCTRL_SIDE_EN |
                                 RP2040_PIO_SM_EXECCTRL_SIDE_PINDIR)) |
                (optional ? RP2040_PIO_SM_EXECCTRL_SIDE_EN : 0) |
                (pindirs ? RP2040_PIO_SM_EXECCTRL_SIDE_PINDIR : 0);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_clkdiv
 *
 * Description:
 *   Set the state machine clock divider (from a floating point value)
 *   in a state machine configuration
 *
 *   The clock divider slows the state machine's execution by masking the
 *   system clock on some cycles, in a repeating pattern, so that the state
 *   machine does not advance. Effectively this produces a slower clock for
 *   the state machine to run from, which can be used to generate e.g. a
 *   particular UART baud rate. See the datasheet for further detail.
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   div - The fractional divisor to be set. 1 for full speed. An integer
 *         clock divisor of n will cause the state machine to run 1 cycle in
 *         every n. Note that for small n, the jitter introduced by a
 *         fractional divider (e.g. 2.5) may be unacceptable although it will
 *         depend on the use case.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_clkdiv(rp2040_pio_sm_config *c,
                                               float div)
{
  uint32_t div_int = (uint32_t)div;
  uint32_t div_frac = (uint32_t)((div - (float)div_int) * (1u << 8u));
  c->clkdiv = (div_frac << RP2040_PIO_SM_CLKDIV_FRAC_SHIFT) |
              (div_int << RP2040_PIO_SM_CLKDIV_INT_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_clkdiv_int_frac
 *
 * Description:
 *   Set the state machine clock divider (from integer and fractional parts
 *   - 16:8) in a state machine configuration
 *
 *   The clock divider can slow the state machine's execution to some rate
 *   below the system clock frequency, by enabling the state machine on some
 *   cycles but not on others, in a regular pattern. This can be used to
 *   generate e.g. a given UART baud rate. See the datasheet for further
 *   detail
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   div_int - Integer part of the divisor
 *   div_frac - Fractional part in 1/256ths
 *
 * See Also:
 *   rp2040_sm_config_set_clkdiv()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_clkdiv_int_frac(
                                            rp2040_pio_sm_config *c,
                                            uint16_t div_int,
                                            uint8_t div_frac)
{
  c->clkdiv = (((uint32_t)div_frac) << RP2040_PIO_SM_CLKDIV_FRAC_SHIFT) |
              (((uint32_t)div_int) << RP2040_PIO_SM_CLKDIV_INT_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_wrap
 *
 * Description:
 *   Set the wrap addresses in a state machine configuration
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   wrap_target - the instruction memory address to wrap to
 *   wrap -        the instruction memory address after which to set the
 *                 program counter to wrap_target if the instruction does not
 *                 itself update the program_counter
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_wrap(rp2040_pio_sm_config *c,
                                             uint32_t wrap_target,
                                             uint32_t wrap)
{
  valid_params_if(PIO, wrap < PIO_INSTRUCTION_COUNT);
  valid_params_if(PIO, wrap_target < PIO_INSTRUCTION_COUNT);
  c->execctrl = (c->execctrl & ~(RP2040_PIO_SM_EXECCTRL_WRAP_TOP_MASK |
                                 RP2040_PIO_SM_EXECCTRL_WRAP_BOTTOM_MASK)) |
                (wrap_target << RP2040_PIO_SM_EXECCTRL_WRAP_BOTTOM_SHIFT) |
                (wrap << RP2040_PIO_SM_EXECCTRL_WRAP_TOP_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_jmp_pin
 *
 * Description:
 *   Set the 'jmp' pin in a state machine configuration
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   pin - The raw GPIO pin number to use as the source for a `jmp pin`
 *            instruction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_jmp_pin(rp2040_pio_sm_config *c,
                                                uint32_t pin)
{
  valid_params_if(PIO, pin < 32);
  c->execctrl = (c->execctrl & ~RP2040_PIO_SM_EXECCTRL_JMP_PIN_MASK) |
                (pin << RP2040_PIO_SM_EXECCTRL_JMP_PIN_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_in_shift
 *
 * Description:
 *   Setup 'in' shifting parameters in a state machine configuration
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   shift_right - true to shift ISR to right, false to shift ISR to left
 *   autopush - whether autopush is enabled
 *   push_threshold - threshold in bits to shift in before auto/conditional
 *                    re-pushing of the ISR
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_in_shift(rp2040_pio_sm_config *c,
                                                 bool shift_right,
                                                 bool autopush,
                                                 uint32_t push_threshold)
{
  valid_params_if(PIO, push_threshold <= 32);
  c->shiftctrl = (c->shiftctrl &
                  ~(RP2040_PIO_SM_SHIFTCTRL_IN_SHIFTDIR |
                    RP2040_PIO_SM_SHIFTCTRL_AUTOPUSH |
                    RP2040_PIO_SM_SHIFTCTRL_PUSH_THRESH_MASK)) |
                 (shift_right ? RP2040_PIO_SM_SHIFTCTRL_IN_SHIFTDIR : 0) |
                 (autopush ? RP2040_PIO_SM_SHIFTCTRL_AUTOPUSH : 0) |
                 ((push_threshold & 0x1fu) <<
                  RP2040_PIO_SM_SHIFTCTRL_PUSH_THRESH_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_out_shift
 *
 * Description:
 *   Setup 'out' shifting parameters in a state machine configuration
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   shift_right - true to shift OSR to right, false to shift OSR to left
 *   autopull - whether autopull is enabled
 *   pull_threshold - threshold in bits to shift out before auto/conditional
 *                    re-pulling of the OSR
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_out_shift(rp2040_pio_sm_config *c,
                                                  bool shift_right,
                                                  bool autopull,
                                                  uint32_t pull_threshold)
{
  valid_params_if(PIO, pull_threshold <= 32);
  c->shiftctrl = (c->shiftctrl &
                  ~(RP2040_PIO_SM_SHIFTCTRL_OUT_SHIFTDIR |
                    RP2040_PIO_SM_SHIFTCTRL_AUTOPULL |
                    RP2040_PIO_SM_SHIFTCTRL_PULL_THRESH_MASK)) |
                 (shift_right ? RP2040_PIO_SM_SHIFTCTRL_OUT_SHIFTDIR : 0) |
                 (autopull ? RP2040_PIO_SM_SHIFTCTRL_AUTOPULL : 0) |
                 ((pull_threshold & 0x1fu) <<
                  RP2040_PIO_SM_SHIFTCTRL_PULL_THRESH_SHIFT);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_fifo_join
 *
 * Description:
 *   Setup the FIFO joining in a state machine configuration
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   join - Specifies the join type.
 *
 * See Also:
 *   enum rp2040_pio_fifo_join
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_fifo_join(rp2040_pio_sm_config *c,
                                          enum rp2040_pio_fifo_join join)
{
  valid_params_if(PIO, join >= RP2040_PIO_FIFO_JOIN_NONE &&
                       join <= RP2040_PIO_FIFO_JOIN_RX);
  c->shiftctrl = (c->shiftctrl &
                  (uint32_t)~(RP2040_PIO_SM_SHIFTCTRL_FJOIN_TX |
                              RP2040_PIO_SM_SHIFTCTRL_FJOIN_RX)) |
                 ((join == RP2040_PIO_FIFO_JOIN_TX) ?
                  RP2040_PIO_SM_SHIFTCTRL_FJOIN_TX : 0) |
                 ((join == RP2040_PIO_FIFO_JOIN_RX) ?
                  RP2040_PIO_SM_SHIFTCTRL_FJOIN_RX : 0);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_out_special
 *
 * Description:
 *   Set special 'out' operations in a state machine configuration
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   sticky - to enable 'sticky' output (i.e. re-asserting most recent
 *            OUT/SET pin values on subsequent cycles)
 *   has -_enable_pin true to enable auxiliary OUT enable pin
 *   enable -_pin_index pin index for auxiliary OUT enable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_out_special(rp2040_pio_sm_config *c,
                                                bool sticky,
                                                bool has_enable_pin,
                                                uint32_t enable_pin_index)
{
  c->execctrl = (c->execctrl &
                 (uint32_t)~(RP2040_PIO_SM_EXECCTRL_OUT_STICKY |
                             RP2040_PIO_SM_EXECCTRL_INLINE_OUT_EN |
                             RP2040_PIO_SM_EXECCTRL_OUT_EN_SEL_MASK)) |
                (sticky ? RP2040_PIO_SM_EXECCTRL_OUT_STICKY : 0) |
                (has_enable_pin ? RP2040_PIO_SM_EXECCTRL_INLINE_OUT_EN : 0) |
                ((enable_pin_index <<
                  RP2040_PIO_SM_EXECCTRL_OUT_EN_SEL_SHIFT)
                 & RP2040_PIO_SM_EXECCTRL_OUT_EN_SEL_MASK);
}

/****************************************************************************
 * Name: rp2040_sm_config_set_mov_status
 *
 * Description:
 *   Set source for 'mov status' in a state machine configuration
 *
 * Input Parameters:
 *   c - Pointer to the configuration structure to modify
 *   status_sel - the status operation selector.
 *   status_n - parameter for the mov status operation (currently a bit
 *              count)
 *
 * See Also:
 *   enum rp2040_pio_mov_status_type
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_sm_config_set_mov_status(rp2040_pio_sm_config *c,
                                  enum rp2040_pio_mov_status_type status_sel,
                                  uint32_t status_n)
{
  valid_params_if(PIO, status_sel >= RP2040_STATUS_TX_LESSTHAN &&
                       status_sel <= RP2040_STATUS_RX_LESSTHAN);
  c->execctrl = (c->execctrl &
                 ~(RP2040_PIO_SM_EXECCTRL_STATUS_SEL |
                   RP2040_PIO_SM_EXECCTRL_STATUS_N_MASK)) |
                ((status_sel == RP2040_STATUS_RX_LESSTHAN) ?
                 RP2040_PIO_SM_EXECCTRL_STATUS_SEL : 0) |
                (status_n & RP2040_PIO_SM_EXECCTRL_STATUS_N_MASK);
}

/****************************************************************************
 * Name: rp2040_pio_get_default_sm_config
 *
 * Description:
 *   Get the default state machine configuration
 *
 *   Setting     | Default
 *   ------------|---------------------------------------------------------
 *   Out Pins    | 32 starting at 0
 *   Set Pins    | 0 starting at 0
 *   In Pins (base)       | 0
 *   Side Set Pins (base) | 0
 *   Side Set    | disabled
 *   Wrap        | wrap=31, wrap_to=0
 *   In Shift    | shift_direction=right, autopush=false, push_thrshold=32
 *   Out Shift   | shift_direction=right, autopull=false, pull_thrshold=32
 *   Jmp Pin     | 0
 *   Out Special | sticky=false, has_enable_pin=false, enable_pin_index=0
 *   Mov Status  | status_sel=STATUS_TX_LESSTHAN, n=0
 *
 * Returned Value:
 *   the default state machine configuration which can then be modified.
 *
 ****************************************************************************/

static inline rp2040_pio_sm_config rp2040_pio_get_default_sm_config(void)
{
  rp2040_pio_sm_config c =
    {
      0, 0, 0, 0
    };
  rp2040_sm_config_set_clkdiv_int_frac(&c, 1, 0);
  rp2040_sm_config_set_wrap(&c, 0, 31);
  rp2040_sm_config_set_in_shift(&c, true, false, 32);
  rp2040_sm_config_set_out_shift(&c, true, false, 32);
  return c;
}

/****************************************************************************
 * Name: rp2040_pio_sm_set_config
 *
 * Description:
 *   Apply a state machine configuration to a state machine
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   config - the configuration to apply
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_set_config(uint32_t pio,
                                uint32_t sm,
                                const rp2040_pio_sm_config *config)
{
  check_pio_param(pio);
  check_sm_param(sm);
  putreg32(config->clkdiv, RP2040_PIO_SM_CLKDIV(pio, sm));
  putreg32(config->execctrl, RP2040_PIO_SM_EXECCTRL(pio, sm));
  putreg32(config->shiftctrl, RP2040_PIO_SM_SHIFTCTRL(pio, sm));
  putreg32(config->pinctrl, RP2040_PIO_SM_PINCTRL(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_get_index
 *
 * Description:
 *   Return the instance number of a PIO instance
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *
 * Returned Value:
 *   the PIO instance number (either 0 or 1)
 *
 ****************************************************************************/

static inline uint32_t rp2040_pio_get_index(uint32_t pio)
{
  check_pio_param(pio);
  return pio;
}

/****************************************************************************
 * Name: rp2040_pio_gpio_init
 *
 * Description:
 *   Setup the function select for a GPIO to use output from the given
 *   PIO instance
 *
 *   PIO appears as an alternate function in the GPIO muxing, just like an
 *   SPI or UART. This function configures that multiplexing to connect a
 *   given PIO instance to a GPIO. Note that this is not necessary for a
 *   state machine to be able to read the *input* value from a GPIO, but
 *   only for it to set the output value or output enable.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   pin - the GPIO pin whose function select to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_gpio_init(uint32_t pio, uint32_t pin)
{
  check_pio_param(pio);
  valid_params_if(PIO, pin < 32);
  rp2040_gpio_set_function(pin, pio == 0 ? RP2040_GPIO_FUNC_PIO0 :
                                           RP2040_GPIO_FUNC_PIO1);
}

/****************************************************************************
 * Name: rp2040_pio_get_dreq
 *
 * Description:
 *   Return the DREQ to use for pacing transfers to a particular state
 *   machine
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   is_tx - true for sending data to the state machine, false for received
 *           data from the state machine
 *
 * Returned Value:
 *   the DREQ number
 *
 ****************************************************************************/

static inline uint32_t rp2040_pio_get_dreq(uint32_t pio, uint32_t sm,
                                           bool is_tx)
{
  check_pio_param(pio);
  check_sm_param(sm);
  return sm + (is_tx ? 0 : RP2040_PIO_SM_NUM) +
              (pio == 0 ? RP2040_DMA_DREQ_PIO0_TX0 :
                          RP2040_DMA_DREQ_PIO1_TX0);
}

/****************************************************************************
 * Name: rp2040_pio_sm_set_enabled
 *
 * Description:
 *   Enable or disable a PIO state machine
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   enabled - true to enable the state machine; false to disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_set_enabled(uint32_t pio,
                                             uint32_t sm,
                                             bool enabled)
{
  check_pio_param(pio);
  check_sm_param(sm);
  putreg32((getreg32(RP2040_PIO_CTRL(pio)) & ~(1u << sm)) |
           (enabled ? (1u << sm) : 0), RP2040_PIO_CTRL(pio));
}

/****************************************************************************
 * Name: rp2040_pio_set_sm_mask_enabled
 *
 * Description:
 *   Enable or disable multiple PIO state machines
 *
 *   Note that this method just sets the enabled state of the state machine;
 *   if now enabled they continue exactly from where they left off.
 *
 * See Also:
 *   rp2040_pio_enable_sm_mask_in_sync() if you wish to enable multiple
 *   state machines and ensure their clock dividers are in sync.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   mask - bit mask of state machine indexes to modify the enabled state of
 *   enabled - true to enable the state machines; false to disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_set_sm_mask_enabled(uint32_t pio,
                                                  uint32_t mask,
                                                  bool enabled)
{
  check_pio_param(pio);
  putreg32((getreg32(RP2040_PIO_CTRL(pio)) & ~mask) |
           (enabled ? mask : 0), RP2040_PIO_CTRL(pio));
}

/****************************************************************************
 * Name: rp2040_pio_sm_restart
 *
 * Description:
 *   Restart a state machine with a known state
 *
 *   This method clears the ISR, shift counters, clock divider counter pin
 *   write flags, delay counter, latched EXEC instruction, and IRQ wait
 *   condition.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_restart(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);
  setbits_reg32(1u << (RP2040_PIO_CTRL_SM_RESTART_SHIFT + sm),
                RP2040_PIO_CTRL(pio));
}

/****************************************************************************
 * Name: rp2040_pio_restart_sm_mask
 *
 * Description:
 *   Restart multiple state machine with a known state
 *
 *   This method clears the ISR, shift counters, clock divider counter pin
 *   write flags, delay counter, latched EXEC instruction, and IRQ wait
 *   condition.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   mask - bit mask of state machine indexes to modify the enabled state of
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_restart_sm_mask(uint32_t pio, uint32_t mask)
{
  check_pio_param(pio);
  setbits_reg32((mask << RP2040_PIO_CTRL_SM_RESTART_SHIFT) &
                RP2040_PIO_CTRL_SM_RESTART_MASK,
                RP2040_PIO_CTRL(pio));
}

/****************************************************************************
 * Name: rp2040_pio_sm_clkdiv_restart
 *
 * Description:
 *   Restart a state machine's clock divider from a phase of 0
 *
 *   Each state machine's clock divider is a free-running piece of hardware,
 *   that generates a pattern of clock enable pulses for the state machine,
 *   based *only* on the configured integer/fractional divisor. The pattern
 *   of running/halted cycles slows the state machine's execution to some
 *   controlled rate.
 *
 *   This function clears the divider's integer and fractional phase
 *   accumulators so that it restarts this pattern from the beginning. It is
 *   called automatically by pio_sm_init() but can also be called at a later
 *   time, when you enable the state machine, to ensure precisely consistent
 *   timing each time you load and run a given PIO program.
 *
 *   More commonly this hardware mechanism is used to synchronise the
 *   execution clocks of multiple state machines
 *   -- see rp2040_pio_clkdiv_restart_sm_mask().
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_clkdiv_restart(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);
  setbits_reg32(1u << (RP2040_PIO_CTRL_CLKDIV_RESTART_SHIFT + sm),
                RP2040_PIO_CTRL(pio));
}

/****************************************************************************
 * Name: rp2040_pio_clkdiv_restart_sm_mask
 *
 * Description:
 *   Restart multiple state machines' clock dividers from a phase of 0.
 *
 *   Each state machine's clock divider is a free-running piece of hardware,
 *   that generates a pattern of clock enable pulses for the state machine,
 *   based *only* on the configured integer/fractional divisor. The pattern
 *   of running/halted cycles slows the state machine's execution to some
 *   controlled rate.
 *
 *   This function simultaneously clears the integer and fractional phase
 *   accumulators of multiple state machines' clock dividers. If these state
 *   machines all have the same integer and fractional divisors configured,
 *   their clock dividers will run in precise deterministic lockstep from
 *   this point.
 *
 *   With their execution clocks synchronised in this way, it is then safe to
 *   e.g. have multiple state machines performing a 'wait irq' on the same
 *   flag, and all clear it on the same cycle.
 *
 *   Also note that this function can be called whilst state machines are
 *   running (e.g. if you have just changed the clock divisors of some state
 *   machines and wish to resynchronise them), and that disabling a state
 *   machine does not halt its clock divider: that is, if multiple state
 *   machines have their clocks synchronised, you can safely disable and
 *   reenable one of the state machines without losing synchronisation.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   mask - bit mask of state machine indexes to modify the enabled state of
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_clkdiv_restart_sm_mask(uint32_t pio,
                                                     uint32_t mask)
{
  check_pio_param(pio);
  setbits_reg32((mask << RP2040_PIO_CTRL_CLKDIV_RESTART_SHIFT) &
                RP2040_PIO_CTRL_CLKDIV_RESTART_MASK,
                RP2040_PIO_CTRL(pio));
}

/****************************************************************************
 * Name: rp2040_pio_enable_sm_mask_in_sync
 *
 * Description:
 *   Enable multiple PIO state machines synchronizing their clock dividers
 *
 *   This is equivalent to calling both pio_set_sm_mask_enabled() and
 *   pio_clkdiv_restart_sm_mask() on the *same* clock cycle. All state
 *   machines specified by 'mask' are started simultaneously and, assuming
 *   they have the same clock divisors, their divided clocks will stay
 *   precisely synchronised.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   mask - bit mask of state machine indexes to modify the enabled state of
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_enable_sm_mask_in_sync(uint32_t pio,
                                                     uint32_t mask)
{
  check_pio_param(pio);
  setbits_reg32((mask << (RP2040_PIO_CTRL_CLKDIV_RESTART_SHIFT) &
                 RP2040_PIO_CTRL_CLKDIV_RESTART_MASK) |
                (mask & RP2040_PIO_CTRL_SM_ENABLE_MASK),
                RP2040_PIO_CTRL(pio));
}

/****************************************************************************
 * Name: rp2040_pio_sm_get_pc
 *
 * Description:
 *   Return the current program counter for a state machine
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   the program counter
 *
 ****************************************************************************/

static inline uint8_t rp2040_pio_sm_get_pc(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);
  return (uint8_t)getreg32(RP2040_PIO_SM_ADDR(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_exec
 *
 * Description:
 *   Immediately execute an instruction on a state machine
 *
 *   This instruction is executed instead of the next instruction in the
 *   normal control flow on the state machine. Subsequent calls to this
 *   method replace the previous executed instruction if it is still
 *   running.
 *
 * See Also:
 *   rp2040_pio_sm_is_exec_stalled() to see if an executed instruction
 *   is still running (i.e. it is stalled on some condition)
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   instr - the encoded PIO instruction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

inline static void rp2040_pio_sm_exec(uint32_t pio, uint32_t sm,
                                      uint32_t instr)
{
  check_pio_param(pio);
  check_sm_param(sm);
  putreg32(instr, RP2040_PIO_SM_INSTR(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_is_exec_stalled
 *
 * Description:
 *   Determine if an instruction set by pio_sm_exec() is stalled executing
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   true if the executed instruction is still running (stalled)
 *
 ****************************************************************************/

static inline bool rp2040_pio_sm_is_exec_stalled(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);
  return getreg32(RP2040_PIO_SM_EXECCTRL(pio, sm)) &
         RP2040_PIO_SM_EXECCTRL_EXEC_STALLED ? true : false;
}

/****************************************************************************
 * Name: rp2040_pio_sm_exec_wait_blocking
 *
 * Description:
 *   Immediately execute an instruction on a state machine and wait for it
 *   to complete
 *
 *   This instruction is executed instead of the next instruction in the
 *   normal control flow on the state machine. Subsequent calls to this
 *   method replace the previous executed instruction if it is still
 *   running.
 *
 * See Also:
 *   rp2040_pio_sm_is_exec_stalled() to see if an executed instruction
 *   is still running (i.e. it is stalled on some condition)
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   instr - the encoded PIO instruction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_exec_wait_blocking(uint32_t pio,
                                                    uint32_t sm,
                                                    uint32_t instr)
{
  check_pio_param(pio);
  check_sm_param(sm);
  rp2040_pio_sm_exec(pio, sm, instr);
  while (rp2040_pio_sm_is_exec_stalled(pio, sm))
    ;
}

/****************************************************************************
 * Name: rp2040_pio_sm_set_wrap
 *
 * Description:
 *   Set the current wrap configuration for a state machine
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   wrap_target - the instruction memory address to wrap to
 *   wrap -        the instruction memory address after which to set the
 *                 program counter to wrap_target if the instruction does not
 *                 itself update the program_counter
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_set_wrap(uint32_t pio,
                                          uint32_t sm,
                                          uint32_t wrap_target,
                                          uint32_t wrap)
{
  check_pio_param(pio);
  check_sm_param(sm);
  valid_params_if(PIO, wrap < PIO_INSTRUCTION_COUNT);
  valid_params_if(PIO, wrap_target < PIO_INSTRUCTION_COUNT);

  putreg32((getreg32(RP2040_PIO_SM_EXECCTRL(pio, sm)) &
            ~(RP2040_PIO_SM_EXECCTRL_WRAP_TOP_MASK |
              RP2040_PIO_SM_EXECCTRL_WRAP_BOTTOM_MASK)) |
           (wrap_target << RP2040_PIO_SM_EXECCTRL_WRAP_BOTTOM_SHIFT) |
           (wrap << RP2040_PIO_SM_EXECCTRL_WRAP_TOP_SHIFT),
           RP2040_PIO_SM_EXECCTRL(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_set_out_pins
 *
 * Description:
 *   Set the current 'out' pins for a state machine
 *   Can overlap with the 'in', 'set' and 'sideset' pins
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   out_base - 0-31 First pin to set as output
 *   out_count - 0-32 Number of pins to set.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_set_out_pins(uint32_t pio,
                                              uint32_t sm,
                                              uint32_t out_base,
                                              uint32_t out_count)
{
  check_pio_param(pio);
  check_sm_param(sm);
  valid_params_if(PIO, out_base < 32);
  valid_params_if(PIO, out_count <= 32);

  putreg32((getreg32(RP2040_PIO_SM_PINCTRL(pio, sm)) &
            ~(RP2040_PIO_SM_PINCTRL_OUT_BASE_MASK |
              RP2040_PIO_SM_PINCTRL_OUT_COUNT_MASK)) |
           (out_base << RP2040_PIO_SM_PINCTRL_OUT_BASE_SHIFT) |
           (out_count << RP2040_PIO_SM_PINCTRL_OUT_COUNT_SHIFT),
           RP2040_PIO_SM_PINCTRL(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_set_set_pins
 *
 * Description:
 *   Set the current 'set' pins for a state machine
 *   Can overlap with the 'in', 'out' and 'sideset' pins
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   set_base - 0-31 First pin to set as
 *   set_count - 0-5 Number of pins to set.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_set_set_pins(uint32_t pio,
                                              uint32_t sm,
                                              uint32_t set_base,
                                              uint32_t set_count)
{
  check_pio_param(pio);
  check_sm_param(sm);
  valid_params_if(PIO, set_base < 32);
  valid_params_if(PIO, set_count <= 5);

  putreg32((getreg32(RP2040_PIO_SM_PINCTRL(pio, sm)) &
            ~(RP2040_PIO_SM_PINCTRL_SET_BASE_MASK |
              RP2040_PIO_SM_PINCTRL_SET_COUNT_MASK)) |
           (set_base << RP2040_PIO_SM_PINCTRL_SET_BASE_SHIFT) |
           (set_count << RP2040_PIO_SM_PINCTRL_SET_COUNT_SHIFT),
           RP2040_PIO_SM_PINCTRL(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_set_in_pins
 *
 * Description:
 *   Set the current 'in' pins for a state machine
 *   Can overlap with the 'out', ''set' and 'sideset' pins
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   in_base - 0-31 First pin to use as input
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_set_in_pins(uint32_t pio,
                                             uint32_t sm,
                                             uint32_t in_base)
{
  check_pio_param(pio);
  check_sm_param(sm);
  valid_params_if(PIO, in_base < 32);

  putreg32((getreg32(RP2040_PIO_SM_PINCTRL(pio, sm)) &
            ~RP2040_PIO_SM_PINCTRL_IN_BASE_MASK) |
           (in_base << RP2040_PIO_SM_PINCTRL_IN_BASE_SHIFT),
           RP2040_PIO_SM_PINCTRL(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_set_sideset_pins
 *
 * Description:
 *   Set the current 'sideset' pins for a state machine
 *   Can overlap with the 'in', 'out' and 'set' pins
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   sideset_base - 0-31 base pin for 'side set'
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_set_sideset_pins(uint32_t pio,
                                                  uint32_t sm,
                                                  uint32_t sideset_base)
{
  check_pio_param(pio);
  check_sm_param(sm);
  valid_params_if(PIO, sideset_base < 32);

  putreg32((getreg32(RP2040_PIO_SM_PINCTRL(pio, sm)) &
            ~RP2040_PIO_SM_PINCTRL_SIDESET_BASE_MASK) |
           (sideset_base << RP2040_PIO_SM_PINCTRL_SIDESET_BASE_SHIFT),
           RP2040_PIO_SM_PINCTRL(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_put
 *
 * Description:
 *   Write a word of data to a state machine's TX FIFO
 *
 *   This is a raw FIFO access that does not check for fullness. If the FIFO
 *   is full, the FIFO contents and state are not affected by the write
 *   attempt. Hardware sets the TXOVER sticky flag for this FIFO in FDEBUG,
 *   to indicate that the system attempted to write to a full FIFO.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   data - the 32 bit data value
 *
 * See Also:
 *   rp2040_pio_sm_put_blocking()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_put(uint32_t pio, uint32_t sm,
                                     uint32_t data)
{
  check_pio_param(pio);
  check_sm_param(sm);

  putreg32(data, RP2040_PIO_TXF(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_get
 *
 * Description:
 *   Read a word of data from a state machine's RX FIFO
 *
 *   This is a raw FIFO access that does not check for emptiness. If the FIFO
 *   is empty, the hardware ignores the attempt to read from the FIFO (the
 *   FIFO remains in an empty state following the read) and the sticky
 *   RXUNDER flag for this FIFO is set in FDEBUG to indicate that the system
 *   tried to read from this FIFO when empty. The data returned by this
 *   function is undefined when the FIFO is empty.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * See Also:
 *   rp2040_pio_sm_get_blocking()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t rp2040_pio_sm_get(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);

  return getreg32(RP2040_PIO_RXF(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_is_rx_fifo_full
 *
 * Description:
 *   Determine if a state machine's RX FIFO is full
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   true if the RX FIFO is full
 *
 ****************************************************************************/

static inline bool rp2040_pio_sm_is_rx_fifo_full(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);

  return (getreg32(RP2040_PIO_FSTAT(pio)) &
          (1u << (RP2040_PIO_FSTAT_RXFULL_SHIFT + sm))) != 0;
}

/****************************************************************************
 * Name: rp2040_pio_sm_is_rx_fifo_empty
 *
 * Description:
 *   Determine if a state machine's RX FIFO is empty
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   true if the RX FIFO is empty
 *
 ****************************************************************************/

static inline bool rp2040_pio_sm_is_rx_fifo_empty(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);

  return (getreg32(RP2040_PIO_FSTAT(pio)) &
          (1u << (RP2040_PIO_FSTAT_RXEMPTY_SHIFT + sm))) != 0;
}

/****************************************************************************
 * Name: rp2040_pio_sm_get_rx_fifo_level
 *
 * Description:
 *   Return the number of elements currently in a state machine's RX FIFO
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   the number of elements in the RX FIFO
 *
 ****************************************************************************/

static inline uint32_t rp2040_pio_sm_get_rx_fifo_level(uint32_t pio,
                                                       uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);

  uint32_t bitoffs = RP2040_PIO_FLEVEL_RX0_SHIFT +
                     sm * (RP2040_PIO_FLEVEL_RX1_SHIFT -
                           RP2040_PIO_FLEVEL_RX0_SHIFT);
  const uint32_t mask = RP2040_PIO_FLEVEL_RX0_MASK >>
                        RP2040_PIO_FLEVEL_RX0_SHIFT;
  return (getreg32(RP2040_PIO_FLEVEL(pio)) >> bitoffs) & mask;
}

/****************************************************************************
 * Name: rp2040_pio_sm_is_tx_fifo_full
 *
 * Description:
 *   Determine if a state machine's TX FIFO is full
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   true if the TX FIFO is full
 *
 ****************************************************************************/

static inline bool rp2040_pio_sm_is_tx_fifo_full(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);

  return (getreg32(RP2040_PIO_FSTAT(pio)) &
          (1u << (RP2040_PIO_FSTAT_TXFULL_SHIFT + sm))) != 0;
}

/****************************************************************************
 * Name: rp2040_pio_sm_is_tx_fifo_empty
 *
 * Description:
 *   Determine if a state machine's TX FIFO is empty
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   true if the TX FIFO is empty
 *
 ****************************************************************************/

static inline bool rp2040_pio_sm_is_tx_fifo_empty(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);

  return (getreg32(RP2040_PIO_FSTAT(pio)) &
          (1u << (RP2040_PIO_FSTAT_TXEMPTY_SHIFT + sm))) != 0;
}

/****************************************************************************
 * Name: rp2040_pio_sm_get_tx_fifo_level
 *
 * Description:
 *   Return the number of elements currently in a state machine's TX FIFO
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   the number of elements in the TX FIFO
 *
 ****************************************************************************/

static inline uint32_t rp2040_pio_sm_get_tx_fifo_level(uint32_t pio,
                                                       uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);

  uint32_t bitoffs = RP2040_PIO_FLEVEL_TX0_SHIFT +
                     sm * (RP2040_PIO_FLEVEL_TX1_SHIFT -
                           RP2040_PIO_FLEVEL_TX0_SHIFT);
  const uint32_t mask = RP2040_PIO_FLEVEL_TX0_MASK >>
                        RP2040_PIO_FLEVEL_TX0_SHIFT;
  return (getreg32(RP2040_PIO_FLEVEL(pio)) >> bitoffs) & mask;
}

/****************************************************************************
 * Name: rp2040_pio_sm_put_blocking
 *
 * Description:
 *   Write a word of data to a state machine's TX FIFO, blocking if the FIFO
 *   is full
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   data - the 32 bit data value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_put_blocking(uint32_t pio,
                                              uint32_t sm,
                                              uint32_t data)
{
  check_pio_param(pio);
  check_sm_param(sm);

  while (rp2040_pio_sm_is_tx_fifo_full(pio, sm))
    ;
  rp2040_pio_sm_put(pio, sm, data);
}

/****************************************************************************
 * Name: rp2040_pio_sm_get_blocking
 *
 * Description:
 *   Read a word of data from a state machine's RX FIFO, blocking if the FIFO
 *   is empty
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t rp2040_pio_sm_get_blocking(uint32_t pio, uint32_t sm)
{
  check_pio_param(pio);
  check_sm_param(sm);

  while (rp2040_pio_sm_is_rx_fifo_empty(pio, sm))
    ;
  return rp2040_pio_sm_get(pio, sm);
}

/****************************************************************************
 * Name: rp2040_pio_sm_set_clkdiv
 *
 * Description:
 *   set the current clock divider for a state machine
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   div - the floating point clock divider
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_set_clkdiv(uint32_t pio, uint32_t sm,
                                            float div)
{
  check_pio_param(pio);
  check_sm_param(sm);

  uint32_t div_int = (uint16_t) div;
  uint32_t div_frac = (uint8_t) ((div - (float)div_int) * (1u << 8u));
  putreg32((div_frac << RP2040_PIO_SM_CLKDIV_FRAC_SHIFT) |
           (div_int << RP2040_PIO_SM_CLKDIV_INT_SHIFT),
           RP2040_PIO_SM_CLKDIV(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_set_clkdiv_int_frac
 *
 * Description:
 *   set the current clock divider for a state machine using a 16:8 fraction
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   div_int - the integer part of the clock divider
 *   div_frac - the fractional part of the clock divider in 1/256s
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_set_clkdiv_int_frac(uint32_t pio,
                                                     uint32_t sm,
                                                     uint16_t div_int,
                                                     uint8_t div_frac)
{
  check_pio_param(pio);
  check_sm_param(sm);

  putreg32((((uint32_t)div_frac) << RP2040_PIO_SM_CLKDIV_FRAC_SHIFT) |
           (((uint32_t)div_int) << RP2040_PIO_SM_CLKDIV_INT_SHIFT),
           RP2040_PIO_SM_CLKDIV(pio, sm));
}

/****************************************************************************
 * Name: rp2040_pio_sm_clear_fifos
 *
 * Description:
 *   Clear a state machine's TX and RX FIFOs
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rp2040_pio_sm_clear_fifos(uint32_t pio, uint32_t sm)
{
  /* changing the FIFO join state clears the fifo */

  check_pio_param(pio);
  check_sm_param(sm);

  xorbits_reg32(RP2040_PIO_SM_SHIFTCTRL_FJOIN_RX,
                RP2040_PIO_SM_SHIFTCTRL(pio, sm));
  xorbits_reg32(RP2040_PIO_SM_SHIFTCTRL_FJOIN_RX,
                RP2040_PIO_SM_SHIFTCTRL(pio, sm));
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_pio_can_add_program
 *
 * Description:
 *   Determine whether the given program can (at the time of the call) be
 *   loaded onto the PIO instance
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   program - the program definition
 *
 * Returned Value:
 *   true if the program can be loaded; false if there is not suitable space
 *   in the instruction memory
 *
 ****************************************************************************/

bool rp2040_pio_can_add_program(uint32_t pio,
                                const rp2040_pio_program_t *program);

/****************************************************************************
 * Name: rp2040_pio_can_add_program_at_offset
 *
 * Description:
 *   Determine whether the given program can (at the time of the call) be
 *   loaded onto the PIO instance starting at a particular location
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   program - the program definition
 *   offset - the instruction memory offset wanted for the start of the
 *            program
 *
 * Returned Value:
 *   true if the program can be loaded at that location; false if there is
 *   not space in the instruction memory
 *
 ****************************************************************************/

bool rp2040_pio_can_add_program_at_offset(uint32_t pio,
                                        const rp2040_pio_program_t *program,
                                        uint32_t offset);

/****************************************************************************
 * Name: rp2040_pio_add_program
 *
 * Description:
 *   Attempt to load the program, panicking if not possible
 *
 * See Also:
 *   rp2040_pio_can_add_program() if you need to check whether the program
 *   can be loaded
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   program - the program definition
 *
 * Returned Value:
 *   the instruction memory offset the program is loaded at
 *
 ****************************************************************************/

uint32_t rp2040_pio_add_program(uint32_t pio,
                                const rp2040_pio_program_t *program);

/****************************************************************************
 * Name: rp2040_pio_add_program_at_offset
 *
 * Description:
 *   Attempt to load the program at the specified instruction memory offset,
 *   panicking if not possible
 *
 * See Also:
 *   rp2040_pio_can_add_program_at_offset() if you need to check whether the
 *   program can be loaded
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   program - the program definition
 *   offset - the instruction memory offset wanted for the start of the
 *            program
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_add_program_at_offset(uint32_t pio,
                                      const rp2040_pio_program_t *program,
                                      uint32_t offset);

/****************************************************************************
 * Name: rp2040_pio_remove_program
 *
 * Description:
 *   Remove a program from a PIO instance's instruction memory
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   program - the program definition
 *   loaded_offset - the loaded offset returned when the program was added
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_remove_program(uint32_t pio,
                               const rp2040_pio_program_t *program,
                               uint32_t loaded_offset);

/****************************************************************************
 * Name: rp2040_pio_clear_instruction_memory
 *
 * Description:
 *   Clears all of a PIO instance's instruction memory
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_clear_instruction_memory(uint32_t pio);

/****************************************************************************
 * Name: rp2040_pio_sm_init
 *
 * Description:
 *   Resets the state machine to a consistent state, and configures it
 *
 *   This method:
 *   - Disables the state machine (if running)
 *   - Clears the FIFOs
 *   - Applies the configuration specified by 'config'
 *   - Resets any internal state e.g. shift counters
 *   - Jumps to the initial program location given by 'initial_pc'
 *
 *   The state machine is left disabled on return from this call.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *   initial_pc - the initial program memory offset to run from
 *   config - the configuration to apply (or NULL to apply defaults)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_sm_init(uint32_t pio, uint32_t sm, uint32_t initial_pc,
                        const rp2040_pio_sm_config *config);

/****************************************************************************
 * Name: rp2040_pio_sm_drain_tx_fifo
 *
 * Description:
 *   Empty out a state machine's TX FIFO
 *
 *   This method executes `pull` instructions on the state machine until the
 *   TX FIFO is empty. This disturbs the contents of the OSR, so see also
 *   pio_sm_clear_fifos() which clears both FIFOs but leaves the state
 *   machine's internal state undisturbed.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * See Also:
 *   rp2040_pio_sm_clear_fifos()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_sm_drain_tx_fifo(uint32_t pio, uint32_t sm);

/****************************************************************************
 * Name: rp2040_pio_sm_set_pins
 *
 * Description:
 *   Use a state machine to set a value on all pins for the PIO instance
 *
 *   This method repeatedly reconfigures the target state machine's pin
 *   configuration and executes 'set' instructions to set values on all 32
 *   pins, before restoring the state machine's pin configuration to what it
 *   was.
 *
 *   This method is provided as a convenience to set initial pin states, and
 *   should not be used against a state machine that is enabled.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3) to use
 *   pin_values - the pin values to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_sm_set_pins(uint32_t pio, uint32_t sm, uint32_t pin_values);

/****************************************************************************
 * Name: rp2040_pio_sm_set_pins_with_mask
 *
 * Description:
 *   Use a state machine to set a value on multiple pins for the PIO instance
 *
 *   This method repeatedly reconfigures the target state machine's pin
 *   configuration and executes 'set' instructions to set values on up to 32
 *   pins, before restoring the state machine's pin configuration to what it
 *   was.
 *
 *   This method is provided as a convenience to set initial pin states, and
 *   should not be used against a state machine that is enabled.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3) to use
 *   pin_values - the pin values to set (if the corresponding bit in pin_mask
 *                is set)
 *   pin_mask - a bit for each pin to indicate whether the corresponding
 *              pin_value for that pin should be applied.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_sm_set_pins_with_mask(uint32_t pio, uint32_t sm,
                                    uint32_t pin_values, uint32_t pin_mask);

/****************************************************************************
 * Name: rp2040_pio_sm_set_pindirs_with_mask
 *
 * Description:
 *   Use a state machine to set the pin directions for multiple pins for the
 *   PIO instance
 *
 *   This method repeatedly reconfigures the target state machine's pin
 *   configuration and executes 'set' instructions to set pin directions on
 *   up to 32 pins, before restoring the state machine's pin configuration to
 *   what it was.
 *
 *   This method is provided as a convenience to set initial pin directions,
 *   and should not be used against a state machine that is enabled.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3) to use
 *   pin_dirs - the pin directions to set - 1 = out, 0 = in (if the
 *              corresponding bit in pin_mask is set)
 *   pin_mask - a bit for each pin to indicate whether the corresponding
 *              pin_value for that pin should be applied.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_sm_set_pindirs_with_mask(uint32_t pio, uint32_t sm,
                                         uint32_t pin_dirs,
                                         uint32_t pin_mask);

/****************************************************************************
 * Name: rp2040_pio_sm_set_consecutive_pindirs
 *
 * Description:
 *   Use a state machine to set the same pin direction for multiple
 *   consecutive pins for the PIO instance
 *
 *   This method repeatedly reconfigures the target state machine's pin
 *   configuration and executes 'set' instructions to set the pin direction
 *   on consecutive pins, before restoring the state machine's pin
 *   configuration to what it was.
 *
 *   This method is provided as a convenience to set initial pin directions,
 *   and should not be used against a state machine that is enabled.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3) to use
 *   pin_base - the first pin to set a direction for
 *   pin_count - the count of consecutive pins to set the direction for
 *   is_out - the direction to set; true = out, false = in
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_sm_set_consecutive_pindirs(uint32_t pio, uint32_t sm,
                                           uint32_t pin_base,
                                           uint32_t pin_count, bool is_out);

/****************************************************************************
 * Name: rp2040_pio_sm_claim
 *
 * Description:
 *   Mark a state machine as used
 *
 *   Method for cooperative claiming of hardware. Will cause a panic if the
 *   state machine is already claimed. Use of this method by libraries
 *   detects accidental configurations that would fail in unpredictable ways.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_sm_claim(uint32_t pio, uint32_t sm);

/****************************************************************************
 * Name: rp2040_pio_claim_sm_mask
 *
 * Description:
 *   Mark multiple state machines as used
 *
 *   Method for cooperative claiming of hardware. Will cause a panic if any
 *   of the state machines are already claimed. Use of this method by
 *   libraries detects accidental configurations that would fail in
 *   unpredictable ways.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm_mask - Mask of state machine indexes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_claim_sm_mask(uint32_t pio, uint32_t sm_mask);

/****************************************************************************
 * Name: rp2040_pio_sm_unclaim
 *
 * Description:
 *   Mark a state machine as no longer used
 *
 * Method for cooperative claiming of hardware.
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   sm - State machine index (0..3)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp2040_pio_sm_unclaim(uint32_t pio, uint32_t sm);

/****************************************************************************
 * Name: rp2040_pio_claim_unused_sm
 *
 * Description:
 *   Claim a free state machine on a PIO instance
 *
 * Input Parameters:
 *   pio - PIO index (0..1)
 *   required - if true the function will panic if none are available
 *
 * Returned Value:
 *   the state machine index or -1 if required was false, and none were free
 *
 ****************************************************************************/

int rp2040_pio_claim_unused_sm(uint32_t pio, bool required);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP2040_RP2040_PIO_H */
