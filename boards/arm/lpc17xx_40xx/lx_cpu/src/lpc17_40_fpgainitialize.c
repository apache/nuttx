/****************************************************************************
 * boards/arm/lpc17xx_40xx/lx_cpu/src/lpc17_40_fpgainitialize.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nuttx/signal.h"
#include "lpc17_40_gpio.h"
#include "arm_internal.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_emc.h"

#include "lx_cpu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
