/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/hardware/spacemit_k1x_rcpu_uart.h
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

#ifndef __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_HARDWARE_SPACEMIT_K1X_RCPU_UART_H
#define __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_HARDWARE_SPACEMIT_K1X_RCPU_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The K1 RCPU console is r_uart0 at 0xc0881000.  It is a 16550 compatible
 * block in the PXA/XScale variant, with two properties that a plain 16550
 * driver does not expect:
 *
 *   1. The registers are one register per 32-bit word, not one per byte.
 *      The device tree describes this as "reg-shift = <2>", which is
 *      CONFIG_16550_REGINCR=4 in NuttX terms.  The offsets below are byte
 *      offsets, i.e. register index times four.
 *
 *   2. The block is held disabled until UART_IER_UUE (bit 6 of IER) is set.
 *      Nothing comes out of the transmitter before that, which is a
 *      particularly confusing failure because every register read back
 *      normally.
 *
 * Register offsets are relative to the base.  The ordering was confirmed
 * against the register map published with the vendor RTOS
 * (bsp/spacemit/platform/n308/k1-x/_register_defination.h in
 * spacemit-com/esos).
 */

#define SPACEMIT_K1X_RCPU_UART_RBR_OFFSET         0x00  /* Receive buffer (read)         */
#define SPACEMIT_K1X_RCPU_UART_THR_OFFSET         0x00  /* Transmit holding (write)      */
#define SPACEMIT_K1X_RCPU_UART_DLL_OFFSET         0x00  /* Divisor latch low (DLAB=1)    */
#define SPACEMIT_K1X_RCPU_UART_IER_OFFSET         0x04  /* Interrupt enable              */
#define SPACEMIT_K1X_RCPU_UART_DLH_OFFSET         0x04  /* Divisor latch high (DLAB=1)   */
#define SPACEMIT_K1X_RCPU_UART_IIR_OFFSET         0x08  /* Interrupt identification      */
#define SPACEMIT_K1X_RCPU_UART_FCR_OFFSET         0x08  /* FIFO control (write)          */
#define SPACEMIT_K1X_RCPU_UART_LCR_OFFSET         0x0c  /* Line control                  */
#define SPACEMIT_K1X_RCPU_UART_MCR_OFFSET         0x10  /* Modem control                 */
#define SPACEMIT_K1X_RCPU_UART_LSR_OFFSET         0x14  /* Line status                   */
#define SPACEMIT_K1X_RCPU_UART_MSR_OFFSET         0x18  /* Modem status                  */
#define SPACEMIT_K1X_RCPU_UART_ISR_OFFSET         0x28  /* PXA interrupt status           */
#define SPACEMIT_K1X_RCPU_UART_FOR_OFFSET         0x2c  /* PXA FIFO occupancy             */
#define SPACEMIT_K1X_RCPU_UART_ABR_OFFSET         0x30  /* PXA autobaud                   */
#define SPACEMIT_K1X_RCPU_UART_ACR_OFFSET         0x34  /* PXA autobaud control           */

/* Interrupt enable register */

#define SPACEMIT_K1X_RCPU_UART_IER_RDA            (1 << 0)  /* Received data available  */
#define SPACEMIT_K1X_RCPU_UART_IER_THRE           (1 << 1)  /* Transmit holding empty   */
#define SPACEMIT_K1X_RCPU_UART_IER_RLS            (1 << 2)  /* Receiver line status     */
#define SPACEMIT_K1X_RCPU_UART_IER_MS             (1 << 3)  /* Modem status             */
#define SPACEMIT_K1X_RCPU_UART_IER_RTOIE          (1 << 4)  /* RX time-out (PXA)        */
#define SPACEMIT_K1X_RCPU_UART_IER_UUE            (1 << 6)  /* Unit enable (PXA)        */

/* Interrupt mode used by the interrupt driven driver.
 *
 * RTOIE is not optional on this part: with only RAVIE set the receiver never
 * raises an interrupt at all (measured: IIR stays NO_INT), so a driver that
 * copies the plain 16550 interrupt enable set appears to receive nothing.
 */

#define SPACEMIT_K1X_RCPU_UART_IER_INTMODE \
  (SPACEMIT_K1X_RCPU_UART_IER_RDA | SPACEMIT_K1X_RCPU_UART_IER_RLS | \
   SPACEMIT_K1X_RCPU_UART_IER_RTOIE | SPACEMIT_K1X_RCPU_UART_IER_UUE)

/* Line control register */

#define SPACEMIT_K1X_RCPU_UART_LCR_WLS(n)         ((n) & 3)   /* Word length select   */
#define SPACEMIT_K1X_RCPU_UART_LCR_STB            (1 << 2)    /* 2 stop bits          */
#define SPACEMIT_K1X_RCPU_UART_LCR_PEN            (1 << 3)    /* Parity enable        */
#define SPACEMIT_K1X_RCPU_UART_LCR_EPS            (1 << 4)    /* Even parity select    */
#define SPACEMIT_K1X_RCPU_UART_LCR_DLAB           (1 << 7)    /* Divisor latch access */

/* Interrupt identification register.
 *
 * The PXA variant encodes the source differently from a plain 16550 and adds
 * a time-out bit; see the driver for how each case is handled.
 */

#define SPACEMIT_K1X_RCPU_UART_IIR_NIP            (1 << 0)  /* 0 = pending      */
#define SPACEMIT_K1X_RCPU_UART_IIR_IID_MASK       (3 << 1)
#define SPACEMIT_K1X_RCPU_UART_IIR_IID_TX         (1 << 1)
#define SPACEMIT_K1X_RCPU_UART_IIR_IID_RDA        (2 << 1)
#define SPACEMIT_K1X_RCPU_UART_IIR_IID_RLS        (3 << 1)
#define SPACEMIT_K1X_RCPU_UART_IIR_TOD            (1 << 3)  /* time out (FIFO)  */

/* Line status register */

#define SPACEMIT_K1X_RCPU_UART_LSR_DR             (1 << 0)  /* Data ready               */
#define SPACEMIT_K1X_RCPU_UART_LSR_THRE           (1 << 5)  /* Transmit holding empty   */
#define SPACEMIT_K1X_RCPU_UART_LSR_TEMT           (1 << 6)  /* Transmitter empty        */

/* Modem control register.
 *
 * OUT2 is not a modem signal here: on this part it gates the transmitter and
 * the interrupt output.  A driver that leaves MCR at its reset value writes
 * THR happily and nothing ever leaves the pin, which is a very convincing
 * imitation of a wiring fault.
 */

#define SPACEMIT_K1X_RCPU_UART_MCR_DTR            (1 << 0)
#define SPACEMIT_K1X_RCPU_UART_MCR_RTS            (1 << 1)
#define SPACEMIT_K1X_RCPU_UART_MCR_OUT1           (1 << 2)
#define SPACEMIT_K1X_RCPU_UART_MCR_OUT2           (1 << 3)
#define SPACEMIT_K1X_RCPU_UART_MCR_LOOP           (1 << 4)

/* FIFO control register */

#define SPACEMIT_K1X_RCPU_UART_FCR_FIFO_EN        (1 << 0)
#define SPACEMIT_K1X_RCPU_UART_FCR_RXCLR          (1 << 1)
#define SPACEMIT_K1X_RCPU_UART_FCR_TXCLR          (1 << 2)
#define SPACEMIT_K1X_RCPU_UART_FCR_TIL            (1 << 3)  /* TX int level (0=half)    */
#define SPACEMIT_K1X_RCPU_UART_FCR_BUS32          (1 << 5)  /* 32 bit peripheral bus    */
#define SPACEMIT_K1X_RCPU_UART_FCR_TRIG(n)        (((n) & 3) << 6)

#endif /* __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_HARDWARE_SPACEMIT_K1X_RCPU_UART_H */
