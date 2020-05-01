/************************************************************************************
 * boards/arm/lpc17xx_40xx/lx_cpu/src/lpc17_40_sdraminitialize.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nuttx/signal.h"
#include "arm_arch.h"
#include "lpc17_40_gpio.h"
#include "arm_internal.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_emc.h"

#include "lx_cpu.h"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void lx_cpu_fpga_initialize(void)
{
  uint32_t regval;

  /* Initialize EMC for FPGA */

  lpc17_40_configgpio(BOARD_XC_PROGRAM_PIN);
  lpc17_40_configgpio(BOARD_XC_DONE_PIN);
  lpc17_40_configgpio(BOARD_XC_INIT_PIN);
  lpc17_40_configgpio(BOARD_XC_SUSPEND_PIN);
  lpc17_40_configgpio(BOARD_XC_RDWR_PIN);

  /* Settings:
   * 32 bus width
   * CS polarity: LOW (ATTENTION: Must match FPGA setup)
   * Byte line state: Reads are only 32 bits
   * Extended wait: off
   * Buffer: disabled
   * Write protection: disabled
   */

  putreg32(0x00000002, LPC17_40_EMC_STATICCONFIG0);

  /* Delays - not measured at this point
   * We're running on 72 MHz, FPGA bus is running on 50 MHz async.
   * Read: 32 cycles
   * Write: 33 cycles
   * Turnaround: 2 cycles (cca. 28 ns)
   */

  putreg32(0x1f, LPC17_40_EMC_STATICWAITRD0);
  putreg32(0x1f, LPC17_40_EMC_STATICWAITWR0);
  putreg32(0x01, LPC17_40_EMC_STATICWAITTURN0);

  /* Shift addresses by 2 (32-bit bus) */

  regval = getreg32(LPC17_40_SYSCON_SCS);
  regval &= ~SYSCON_SCS_EMCSC;
  putreg32(regval, LPC17_40_SYSCON_SCS);
}
