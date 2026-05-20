/****************************************************************************
 * arch/arm64/src/am62x/am62x_16550serial.c
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

#ifdef CONFIG_16550_UART

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/serial/uart_16550.h>

#include "arm64_arch.h"
#include "arm64_internal.h"

/* TI AM62x-specific register byte offsets and bits not in uart_16550.h */

#define AM62X_UART_IER_OFF   0x04  /* IER byte offset */
#define AM62X_UART_FCR_OFF   0x08  /* FCR byte offset */
#define AM62X_UART_LSR_OFF   0x14  /* LSR byte offset */
#define AM62X_UART_EFR2_OFF  0x8c  /* EFR2 byte offset (K3 extension) */

/* K3 Errata i2310: resets RX timeout counter on each FIFO read */

#define AM62X_EFR2_TIMEOUT_BEHAVE  (1 << 6)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Wait for the TX FIFO to drain completely before touching FCR.
 * Called before am62x_uart_hw_init to ensure TXRST doesn't discard
 * characters that are still in the FIFO (e.g. debug prints from boot).
 */

static void am62x_uart_drain_tx(uintptr_t base)
{
  int tries = 1000000;

  while (tries-- > 0 &&
         !(getreg32(base + AM62X_UART_LSR_OFF) & UART_LSR_TEMT))
    {
    }
}

/* TI AM62x-specific UART hardware initialisation.
 *
 * 1. TX drain: wait for TX shift register empty (TEMT) so TXRST does not
 *    discard characters still in flight.
 * 2. FCR 3-step: TI K3 UARTs require FIFOEN asserted first, then
 *    RXRST|TXRST, then the desired trigger level.  The generic 16550
 *    sequence (RXRST|TXRST without FIFOEN) disables the FIFO on TI
 *    hardware.
 * 3. RX trigger = 0 (1-char): RDA fires on every received byte, keeping
 *    the FIFO empty and preventing the spurious CTI path (K3 i2310).
 * 4. EFR2 TIMEOUT_BEHAVE (bit 6, offset 0x8c): resets the RX timeout
 *    counter each time data is read from the FIFO, so a post-drain CTI
 *    never fires with an empty FIFO.
 *
 * MDR1 is intentionally not touched -- U-Boot has already placed the UART
 * in 16x mode and writing MDR1 causes THRE to read 0 for several µs.
 */

static void am62x_uart_hw_init(uintptr_t base)
{
  am62x_uart_drain_tx(base);

  putreg32(UART_FCR_FIFOEN,
           base + AM62X_UART_FCR_OFF);

  putreg32(UART_FCR_FIFOEN | UART_FCR_RXRST | UART_FCR_TXRST,
           base + AM62X_UART_FCR_OFF);

  putreg32(UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_1,
           base + AM62X_UART_FCR_OFF);

  /* Keep the UART quiet until the full 16550 driver attaches later in
   * up_initialize().  U-Boot can leave UART6 interrupt sources armed on
   * PocketBeagle2, which causes an immediate trap as soon as NuttX enables
   * global interrupts in irq_initialize().
   */

  putreg32(0, base + AM62X_UART_IER_OFF);
  putreg32(AM62X_EFR2_TIMEOUT_BEHAVE, base + AM62X_UART_EFR2_OFF);
}

static void am62x_uartirq_setup(void)
{
#ifdef CONFIG_16550_UART0
#  ifdef CONFIG_ARCH_IRQPRIO
  (void)up_prioritize_irq(CONFIG_16550_UART0_IRQ, 0);
#  endif

#  ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
  /* TI's AM62 main-domain UARTs are level-high SPIs in the upstream
   * device tree. PocketBeagle2 uses UART6 (GIC_SPI 184 -> INTID 216).
   */

  (void)up_set_irq_type(CONFIG_16550_UART0_IRQ, IRQ_HIGH_LEVEL);
#  endif
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm64_earlyserialinit(void)
{
  /* U-Boot already leaves the AM62x console UART configured at 115200 8N1.
   * Keep that early console state intact until /dev/console is opened, but
   * apply the TI-specific FIFO sequence and timeout behavior immediately so
   * interactive input is safe as soon as the full 16550 driver takes over.
   */

  am62x_uart_hw_init(CONFIG_16550_UART0_BASE);
  u16550_earlyserialinit();
}

void arm64_serialinit(void)
{
  am62x_uartirq_setup();
  u16550_serialinit();
}

#endif /* CONFIG_16550_UART */
