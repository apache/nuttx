/****************************************************************************
 * arch/arm64/src/am62x/hardware/am62x_uart.h
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

/* Reference: AM62x TRM (SPRSP43), Chapter 12 - UART.
 * The AM62x UART is a 16550-compatible peripheral with TI K3 extensions.
 * Register layout matches the industry-standard 16550 at offsets
 * 0x00-0x1c.  TI-specific registers begin at 0x20 (MDR1) and beyond.
 */

#ifndef __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_UART_H
#define __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART register offsets (standard 16550 layout) ****************************/

#define UART_RHR_OFFSET         0x00  /* Receiver Holding Register (read)  */
#define UART_THR_OFFSET         0x00  /* Transmitter Holding Register (wr) */
#define UART_IER_OFFSET         0x04  /* Interrupt Enable Register         */
#define UART_IIR_OFFSET         0x08  /* Interrupt Identification Reg (rd) */
#define UART_FCR_OFFSET         0x08  /* FIFO Control Register (write)     */
#define UART_EFR_OFFSET         0x08  /* Enhanced Feature Register (Mode B)*/
#define UART_LCR_OFFSET         0x0c  /* Line Control Register             */
#define UART_MCR_OFFSET         0x10  /* Modem Control Register            */
#define UART_LSR_OFFSET         0x14  /* Line Status Register              */
#define UART_MSR_OFFSET         0x18  /* Modem Status Register             */
#define UART_DLL_OFFSET         0x00  /* Divisor Latch Low  (LCR[7]=1)     */
#define UART_DLH_OFFSET         0x04  /* Divisor Latch High (LCR[7]=1)     */

/* UART register offsets (TI-specific extensions) ***************************/

#define UART_MDR1_OFFSET        0x20  /* Mode Definition Register 1        */
#define UART_SYSC_OFFSET        0x54  /* System Configuration Register     */
#define UART_SYSS_OFFSET        0x58  /* System Status Register            */
#define UART_RFL_OFFSET         0x64  /* RX FIFO level register            */
#define UART_EFR2_OFFSET        0x8c  /* Enhanced Feature Register 2 (K3)  */
#define UART_TO_L_OFFSET        0x98  /* RX timeout low register  (K3)     */
#define UART_TO_H_OFFSET        0x9c  /* RX timeout high register (K3)     */

/* IER bits *****************************************************************/

#define UART_IER_RHR            (1 << 0)  /* RX data available + CTI       */
#define UART_IER_THR            (1 << 1)  /* TX holding register empty     */
#define UART_IER_LINE           (1 << 2)  /* Receiver line status          */
#define UART_IER_MODEM          (1 << 3)  /* Modem status                  */

/* IIR bits (read) **********************************************************/

#define UART_IIR_PENDING        (1 << 0)  /* 1 = no interrupt pending      */
#define UART_IIR_ID_MASK        0x3e      /* Interrupt ID bits [5:1]       */
#define UART_IIR_ID_RLS         0x06      /* Receiver line status          */
#define UART_IIR_ID_RDA         0x04      /* Received data available       */
#define UART_IIR_ID_CTI         0x0c      /* Character timeout indication  */
#define UART_IIR_ID_THRE        0x02      /* THR empty                     */
#define UART_IIR_ID_MSR         0x00      /* Modem status                  */

/* FCR bits *****************************************************************/

#define UART_FCR_FIFOEN         (1 << 0)  /* Enable TX/RX FIFOs            */
#define UART_FCR_RXRST          (1 << 1)  /* Reset RX FIFO                 */
#define UART_FCR_TXRST          (1 << 2)  /* Reset TX FIFO                 */
#define UART_FCR_TXTRIG_SHIFT   4
#define UART_FCR_RXTRIG_SHIFT   6
#define UART_FCR_TXTRIG(n)      (((n) & 3) << UART_FCR_TXTRIG_SHIFT)
#define UART_FCR_RXTRIG(n)      (((n) & 3) << UART_FCR_RXTRIG_SHIFT)

/* LCR bits *****************************************************************/

#define UART_LCR_WLS_5          0x00      /* 5-bit word length             */
#define UART_LCR_WLS_6          0x01      /* 6-bit word length             */
#define UART_LCR_WLS_7          0x02      /* 7-bit word length             */
#define UART_LCR_WLS_8          0x03      /* 8-bit word length             */
#define UART_LCR_STB            (1 << 2)  /* 2 stop bits                   */
#define UART_LCR_PEN            (1 << 3)  /* Parity enable                 */
#define UART_LCR_EPS            (1 << 4)  /* Even parity select            */
#define UART_LCR_BRK            (1 << 6)  /* Break control                 */
#define UART_LCR_CONFIG_MODE_A  0x80      /* TI config mode A (DLAB=1)     */
#define UART_LCR_CONFIG_MODE_B  0xbf      /* TI config mode B (EFR access) */

/* MCR bits *****************************************************************/

#define UART_MCR_DTR            (1 << 0)  /* Data terminal ready           */
#define UART_MCR_RTS            (1 << 1)  /* Request to send               */
#define UART_MCR_OUT2           (1 << 3)  /* IRQ output enable (OUT2)      */

/* LSR bits *****************************************************************/

#define UART_LSR_DR             (1 << 0)  /* Data ready in RX FIFO         */
#define UART_LSR_OE             (1 << 1)  /* Overrun error                 */
#define UART_LSR_PE             (1 << 2)  /* Parity error                  */
#define UART_LSR_FE             (1 << 3)  /* Framing error                 */
#define UART_LSR_BI             (1 << 4)  /* Break interrupt               */
#define UART_LSR_THRE           (1 << 5)  /* TX holding register empty     */
#define UART_LSR_TEMT           (1 << 6)  /* TX shift register empty       */
#define UART_LSR_RXFE           (1 << 7)  /* RX FIFO error                 */

/* EFR bits *****************************************************************/

#define UART_EFR_ENHANCEDEN     (1 << 4)  /* Enable enhanced functions     */

/* EFR2 bits (K3-specific) **************************************************/

/* K3 Errata i2310: set this bit to allow clearing a spurious CTI that
 * fires with an empty RX FIFO.  The bit changes how the timeout counter
 * resets; momentarily asserting it together with a max timeout value and
 * reading IIR deasserts the stuck interrupt.
 */

#define UART_EFR2_TIMEOUT_BEHAVE (1 << 6)

/* MDR1 bits ****************************************************************/

#define UART_MDR1_MODE_16X      0x00      /* UART 16x mode (normal)        */
#define UART_MDR1_MODE_DISABLE  0x07      /* Disable UART                  */

/* SYSC bits ****************************************************************/

#define UART_SYSC_SOFTRESET     (1 << 1)  /* Software reset                */
#define UART_SYSC_IDLEMODE_NO   (1 << 3)  /* No-idle mode                  */

/* SYSS bits ****************************************************************/

#define UART_SYSS_RESETDONE     (1 << 0)  /* Reset complete                */

/* UART functional clock.
 * AM62x UART is clocked from HSDIV4_CLKOUT1, configured by TIFS to 48 MHz.
 * Reference: AM62x TRM section 12.3.1 and TIFS documentation.
 */

#define AM62X_UART_SCLK         48000000ul

/* UART IRQ numbers (raw GIC INTIDs = GIC_SPI_number + 32).
 * Source: Linux kernel arch/arm64/boot/dts/ti/k3-am62.dtsi.
 *   UART0 → GIC_SPI 178 → INTID 210
 *   UART6 → GIC_SPI 184 → INTID 216
 */

#define AM62X_UART0_IRQ         210   /* GIC_SPI 178 */
#define AM62X_UART1_IRQ         211   /* GIC_SPI 179 */
#define AM62X_UART2_IRQ         212   /* GIC_SPI 180 */
#define AM62X_UART3_IRQ         213   /* GIC_SPI 181 */
#define AM62X_UART4_IRQ         214   /* GIC_SPI 182 */
#define AM62X_UART5_IRQ         215   /* GIC_SPI 183 */
#define AM62X_UART6_IRQ         216   /* GIC_SPI 184 */

#endif /* __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_UART_H */
