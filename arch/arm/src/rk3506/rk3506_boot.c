/****************************************************************************
 * arch/arm/src/rk3506/rk3506_boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "arm_internal.h"

#include "rk3506_irq.h"
#include "rk3506_memorymap.h"
#include "gic.h"

#ifdef CONFIG_SCHED_INSTRUMENTATION
#  include <sched/sched.h>
#  include <nuttx/sched_note.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RK3506 CRU: force UART4 source clock to the 24MHz crystal (XIN_OSC0).
 *
 * This mirrors what the Rockchip HAL does for RT-Thread, which is the proven
 * configuration that yields a working 1.5Mbaud console on UART4.  NuttX does
 * not otherwise touch the CRU, so without this the UART4 source clock would
 * stay at the (lower) reset/U-Boot value and the 16550 divisor would produce
 * the wrong baud rate.
 *
 *   CRU base                = 0xff9a0000
 *   SCLK_UART4_SEL/_DIV     = CLKSEL_CON31 (offset 0x37c)
 *     - mux  bits[15:13]: 0 = XIN_OSC0 (24MHz)
 *     - div  bits[12:8] : 0 = divide by 1
 *   Rockchip CRU registers use the upper 16 bits as a write-enable mask for
 *   the corresponding lower 16 bits.  Writing 0xff000000 enables bits[15:8]
 *   and clears them -> mux=0, div=0 -> UART4 source = 24MHz.
 */

#define RK3506_CRU_BASE          0xff9a0000
#define RK3506_CRU_CLKSEL_CON31  (RK3506_CRU_BASE + 0x37c)
#define RK3506_UART4_CLK_24M     0xff000000

/* Raw UART4 (dw-apb 16550, reg-shift=2) breadcrumb for early boot.
 * Proves arm_boot() runs and the UART hardware works, independent of
 * the NuttX serial driver / OS init.  Register index << 2 (reg-shift=2).
 */

#define RK3506_UART4_BASE        0xff0e0000
#define UART4_REG(idx) \
        (*(volatile uint32_t *)(RK3506_UART4_BASE + ((idx) << 2)))
#define UART4_RBR_THR_DLL        UART4_REG(0)  /* THR(w) / DLL(DLAB=1) */
#define UART4_IER_DLM            UART4_REG(1)  /* IER / DLM(DLAB=1)    */
#define UART4_FCR                UART4_REG(2)  /* FIFO control         */
#define UART4_LCR                UART4_REG(3)  /* Line control         */
#define UART4_LSR                UART4_REG(5)  /* Line status          */
#define UART4_LSR_THRE           (1 << 5)      /* TX holding empty     */

static void rk3506_rawuart_init(void)
{
  /* 24MHz / (16 * 1) = 1.5Mbaud, matching the working RT-Thread config. */

  UART4_LCR = 0x80;            /* DLAB=1 */
  UART4_RBR_THR_DLL = 0x01;    /* DLL = 1 */
  UART4_IER_DLM = 0x00;        /* DLM = 0 */
  UART4_LCR = 0x03;            /* DLAB=0, 8N1 */
  UART4_FCR = 0x07;            /* enable+clear FIFOs */
}

void rk3506_rawuart_puts(const char *s)
{
  while (*s)
    {
      while ((UART4_LSR & UART4_LSR_THRE) == 0);
      UART4_RBR_THR_DLL = (uint32_t)*s++;
    }
}

/* Debug marker: print a tag repeatedly so it survives the board-powered
 * CH340 USB-serial re-enumeration window (2-3s) on every board reset.
 */

void rk3506_dbgmark(const char *tag)
{
  int i;

  /* Re-assert the known-good 1.5Mbaud divisor before printing. By the time
   * the OS reaches later init stages, the u16550 serial driver may have
   * reconfigured UART4's divisor, which would garble these raw markers.
   */

  rk3506_rawuart_init();

  for (i = 0; i < 30; i++)
    {
      rk3506_rawuart_puts(tag);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 ****************************************************************************/

void arm_boot(void)
{
#ifdef CONFIG_ARCH_PERF_EVENTS
  /* Perf init */

  up_perf_init(0);
#endif

#ifdef CONFIG_ARCH_ARMV7A
  /* Set the page table for section */

  rk3506_setupmappings();
#endif

  arm_fpuconfig();

  /* Force UART4 source clock to 24MHz before bringing up the console. */

  putreg32(RK3506_UART4_CLK_24M, RK3506_CRU_CLKSEL_CON31);

  /* Early boot breadcrumb (single line) before the serial driver comes up. */

  rk3506_rawuart_init();
  rk3506_rawuart_puts("\r\n[RK3506] NuttX arm_boot reached\r\n");

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  arm_earlyserialinit();
#endif
}
