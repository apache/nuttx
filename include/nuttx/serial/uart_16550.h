/****************************************************************************
 * include/nuttx/serial/uart_16550.h
 * Serial driver for 16550 UART
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

#ifndef __INCLUDE_NUTTX_SERIAL_UART_16550_H
#define __INCLUDE_NUTTX_SERIAL_UART_16550_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_16550_UART

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIGURATION ************************************************************/

/* Are any UARTs enabled? */

#undef HAVE_UART
#if defined(CONFIG_16550_UART0) || defined(CONFIG_16550_UART1) || \
    defined(CONFIG_16550_UART2) || defined(CONFIG_16550_UART3)
#  define HAVE_UART 1
#endif

/* We need to be told the address increment between registers and the
 * register bit width.
 */

#ifndef CONFIG_16550_REGINCR
#  error "CONFIG_16550_REGINCR not defined"
#endif

#if CONFIG_16550_REGINCR != 1 && CONFIG_16550_REGINCR != 2 && CONFIG_16550_REGINCR != 4
#  error "CONFIG_16550_REGINCR not supported"
#endif

#ifndef CONFIG_16550_REGWIDTH
#  error "CONFIG_16550_REGWIDTH not defined"
#endif

#if CONFIG_16550_REGWIDTH != 8 && CONFIG_16550_REGWIDTH != 16 && CONFIG_16550_REGWIDTH != 32
#  error "CONFIG_16550_REGWIDTH not supported"
#endif

#ifndef CONFIG_16550_ADDRWIDTH
#  error "CONFIG_16550_ADDRWIDTH not defined"
#endif

#if CONFIG_16550_ADDRWIDTH != 0 && CONFIG_16550_ADDRWIDTH != 8 && \
    CONFIG_16550_ADDRWIDTH != 16 && CONFIG_16550_ADDRWIDTH != 32 && \
    CONFIG_16550_ADDRWIDTH != 64
#  error "CONFIG_16550_ADDRWIDTH not supported"
#endif

/* If a UART is enabled, then its base address, clock, and IRQ
 * must also be provided
 */

#ifdef CONFIG_16550_UART0
#  ifndef CONFIG_16550_UART0_BASE
#    error "CONFIG_16550_UART0_BASE not provided"
#    undef CONFIG_16550_UART0
#  endif
#  ifndef CONFIG_16550_UART0_CLOCK
#    error "CONFIG_16550_UART0_CLOCK not provided"
#    undef CONFIG_16550_UART0
#  endif
#  ifndef CONFIG_16550_UART0_IRQ
#    error "CONFIG_16550_UART0_IRQ not provided"
#    undef CONFIG_16550_UART0
#  endif
#endif

#ifdef CONFIG_16550_UART1
#  ifndef CONFIG_16550_UART1_BASE
#    error "CONFIG_16550_UART1_BASE not provided"
#    undef CONFIG_16550_UART1
#  endif
#  ifndef CONFIG_16550_UART1_CLOCK
#    error "CONFIG_16550_UART1_CLOCK not provided"
#    undef CONFIG_16550_UART1
#  endif
#  ifndef CONFIG_16550_UART1_IRQ
#    error "CONFIG_16550_UART1_IRQ not provided"
#    undef CONFIG_16550_UART1
#  endif
#endif

#ifdef CONFIG_16550_UART2
#  ifndef CONFIG_16550_UART2_BASE
#    error "CONFIG_16550_UART2_BASE not provided"
#    undef CONFIG_16550_UART2
#  endif
#  ifndef CONFIG_16550_UART2_CLOCK
#    error "CONFIG_16550_UART2_CLOCK not provided"
#    undef CONFIG_16550_UART2
#  endif
#  ifndef CONFIG_16550_UART2_IRQ
#    error "CONFIG_16550_UART2_IRQ not provided"
#    undef CONFIG_16550_UART2
#  endif
#endif

#ifdef CONFIG_16550_UART3
#  ifndef CONFIG_16550_UART3_BASE
#    error "CONFIG_16550_UART3_BASE not provided"
#    undef CONFIG_16550_UART3
#  endif
#  ifndef CONFIG_16550_UART3_CLOCK
#    error "CONFIG_16550_UART3_CLOCK not provided"
#    undef CONFIG_16550_UART3
#  endif
#  ifndef CONFIG_16550_UART3_IRQ
#    error "CONFIG_16550_UART3_IRQ not provided"
#    undef CONFIG_16550_UART3
#  endif
#endif

/* Is there a serial console? There should be at most one defined.
 * It could be on any UARTn, n=0,1,2,3
 */

#if defined(CONFIG_16550_UART0_SERIAL_CONSOLE) && defined(CONFIG_16550_UART0)
#  undef CONFIG_16550_UART1_SERIAL_CONSOLE
#  undef CONFIG_16550_UART2_SERIAL_CONSOLE
#  undef CONFIG_16550_UART3_SERIAL_CONSOLE
#  define HAVE_16550_CONSOLE 1
#elif defined(CONFIG_16550_UART1_SERIAL_CONSOLE) && defined(CONFIG_16550_UART1)
#  undef CONFIG_16550_UART0_SERIAL_CONSOLE
#  undef CONFIG_16550_UART2_SERIAL_CONSOLE
#  undef CONFIG_16550_UART3_SERIAL_CONSOLE
#  define HAVE_16550_CONSOLE 1
#elif defined(CONFIG_16550_UART2_SERIAL_CONSOLE) && defined(CONFIG_16550_UART2)
#  undef CONFIG_16550_UART0_SERIAL_CONSOLE
#  undef CONFIG_16550_UART1_SERIAL_CONSOLE
#  undef CONFIG_16550_UART3_SERIAL_CONSOLE
#  define HAVE_16550_CONSOLE 1
#elif defined(CONFIG_16550_UART3_SERIAL_CONSOLE) && defined(CONFIG_16550_UART3)
#  undef CONFIG_16550_UART0_SERIAL_CONSOLE
#  undef CONFIG_16550_UART1_SERIAL_CONSOLE
#  undef CONFIG_16550_UART2_SERIAL_CONSOLE
#  define HAVE_16550_CONSOLE 1
#else
#  undef CONFIG_16550_UART0_SERIAL_CONSOLE
#  undef CONFIG_16550_UART1_SERIAL_CONSOLE
#  undef CONFIG_16550_UART2_SERIAL_CONSOLE
#  undef CONFIG_16550_UART3_SERIAL_CONSOLE
#  undef HAVE_16550_CONSOLE
#endif

/* Register offsets *********************************************************/

#define UART_RBR_INCR          0 /* (DLAB =0) Receiver Buffer Register */
#define UART_THR_INCR          0 /* (DLAB =0) Transmit Holding Register */
#define UART_DLL_INCR          0 /* (DLAB =1) Divisor Latch LSB */
#define UART_DLM_INCR          1 /* (DLAB =1) Divisor Latch MSB */
#define UART_IER_INCR          1 /* (DLAB =0) Interrupt Enable Register */
#define UART_IIR_INCR          2 /* Interrupt ID Register */
#define UART_FCR_INCR          2 /* FIFO Control Register */
#define UART_LCR_INCR          3 /* Line Control Register */
#define UART_MCR_INCR          4 /* Modem Control Register */
#define UART_LSR_INCR          5 /* Line Status Register */
#define UART_MSR_INCR          6 /* Modem Status Register */
#define UART_SCR_INCR          7 /* Scratch Pad Register */

#define UART_RBR_OFFSET        (CONFIG_16550_REGINCR*UART_RBR_INCR)
#define UART_THR_OFFSET        (CONFIG_16550_REGINCR*UART_THR_INCR)
#define UART_DLL_OFFSET        (CONFIG_16550_REGINCR*UART_DLL_INCR)
#define UART_DLM_OFFSET        (CONFIG_16550_REGINCR*UART_DLM_INCR)
#define UART_IER_OFFSET        (CONFIG_16550_REGINCR*UART_IER_INCR)
#define UART_IIR_OFFSET        (CONFIG_16550_REGINCR*UART_IIR_INCR)
#define UART_FCR_OFFSET        (CONFIG_16550_REGINCR*UART_FCR_INCR)
#define UART_LCR_OFFSET        (CONFIG_16550_REGINCR*UART_LCR_INCR)
#define UART_MCR_OFFSET        (CONFIG_16550_REGINCR*UART_MCR_INCR)
#define UART_LSR_OFFSET        (CONFIG_16550_REGINCR*UART_LSR_INCR)
#define UART_MSR_OFFSET        (CONFIG_16550_REGINCR*UART_MSR_INCR)
#define UART_SCR_OFFSET        (CONFIG_16550_REGINCR*UART_SCR_INCR)

/* Register bit definitions *************************************************/

/* RBR (DLAB =0) Receiver Buffer Register */

#define UART_RBR_MASK                (0xff)    /* Bits 0-7: Oldest received byte in RX FIFO */
                                               /* Bits 8-31: Reserved */

/* THR (DLAB =0) Transmit Holding Register */

#define UART_THR_MASK                (0xff)    /* Bits 0-7: Adds byte to TX FIFO */
                                               /* Bits 8-31: Reserved */

/* DLL (DLAB =1) Divisor Latch LSB */

#define UART_DLL_MASK                (0xff)    /* Bits 0-7: DLL */
                                               /* Bits 8-31: Reserved */

/* DLM (DLAB =1) Divisor Latch MSB */

#define UART_DLM_MASK                (0xff)    /* Bits 0-7: DLM */
                                               /* Bits 8-31: Reserved */

/* IER (DLAB =0) Interrupt Enable Register */

#define UART_IER_ERBFI               (1 << 0)  /* Bit 0: Enable received data available interrupt */
#define UART_IER_ETBEI               (1 << 1)  /* Bit 1: Enable THR empty interrupt */
#define UART_IER_ELSI                (1 << 2)  /* Bit 2: Enable receiver line status interrupt */
#define UART_IER_EDSSI               (1 << 3)  /* Bit 3: Enable MODEM status interrupt */
                                               /* Bits 4-7: Reserved */
#define UART_IER_ALLIE               (0x0f)

/* IIR Interrupt ID Register */

#define UART_IIR_INTSTATUS           (1 << 0)  /* Bit 0:  Interrupt status (active low) */
#define UART_IIR_INTID_SHIFT         (1)       /* Bits 1-3: Interrupt identification */
#define UART_IIR_INTID_MASK          (7 << UART_IIR_INTID_SHIFT)
#  define UART_IIR_INTID_MSI         (0 << UART_IIR_INTID_SHIFT) /* Modem Status */
#  define UART_IIR_INTID_THRE        (1 << UART_IIR_INTID_SHIFT) /* THR Empty Interrupt */
#  define UART_IIR_INTID_RDA         (2 << UART_IIR_INTID_SHIFT) /* Receive Data Available (RDA) */
#  define UART_IIR_INTID_RLS         (3 << UART_IIR_INTID_SHIFT) /* Receiver Line Status (RLS) */
#  define UART_IIR_INTID_CTI         (6 << UART_IIR_INTID_SHIFT) /* Character Time-out Indicator (CTI) */

                                               /* Bits 4-5: Reserved */
#define UART_IIR_FIFOEN_SHIFT        (6)       /* Bits 6-7: RCVR FIFO interrupt */
#define UART_IIR_FIFOEN_MASK         (3 << UART_IIR_FIFOEN_SHIFT)

/* FCR FIFO Control Register */

#define UART_FCR_FIFOEN              (1 << 0)  /* Bit 0:  Enable FIFOs */
#define UART_FCR_RXRST               (1 << 1)  /* Bit 1:  RX FIFO Reset */
#define UART_FCR_TXRST               (1 << 2)  /* Bit 2:  TX FIFO Reset */
#define UART_FCR_DMAMODE             (1 << 3)  /* Bit 3:  DMA Mode Select */
                                               /* Bits 4-5: Reserved */
#define UART_FCR_RXTRIGGER_SHIFT     (6)       /* Bits 6-7: RX Trigger Level */
#define UART_FCR_RXTRIGGER_MASK      (3 << UART_FCR_RXTRIGGER_SHIFT)
#  define UART_FCR_RXTRIGGER_1       (0 << UART_FCR_RXTRIGGER_SHIFT) /*  Trigger level 0 (1 character) */
#  define UART_FCR_RXTRIGGER_4       (1 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 1 (4 characters) */
#  define UART_FCR_RXTRIGGER_8       (2 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 2 (8 characters) */
#  define UART_FCR_RXTRIGGER_14      (3 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 3 (14 characters) */

/* LCR Line Control Register */

#define UART_LCR_WLS_SHIFT           (0)       /* Bit 0-1: Word Length Select */
#define UART_LCR_WLS_MASK            (3 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_5BIT          (0 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_6BIT          (1 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_7BIT          (2 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_8BIT          (3 << UART_LCR_WLS_SHIFT)
#define UART_LCR_STB                 (1 << 2)  /* Bit 2:  Number of Stop Bits */
#define UART_LCR_PEN                 (1 << 3)  /* Bit 3:  Parity Enable */
#define UART_LCR_EPS                 (1 << 4)  /* Bit 4:  Even Parity Select */
#define UART_LCR_STICKY              (1 << 5)  /* Bit 5:  Stick Parity */
#define UART_LCR_BRK                 (1 << 6)  /* Bit 6: Break Control */
#define UART_LCR_DLAB                (1 << 7)  /* Bit 7: Divisor Latch Access Bit (DLAB) */

/* MCR Modem Control Register */

#define UART_MCR_DTR                 (1 << 0)  /* Bit 0:  DTR Control Source for DTR output */
#define UART_MCR_RTS                 (1 << 1)  /* Bit 1:  Control Source for  RTS output */
#define UART_MCR_OUT1                (1 << 2)  /* Bit 2:  Auxiliary user-defined output 1 */
#define UART_MCR_OUT2                (1 << 3)  /* Bit 3:  Auxiliary user-defined output 2 */
#define UART_MCR_LPBK                (1 << 4)  /* Bit 4:  Loopback Mode Select */
#define UART_MCR_AFCE                (1 << 5)  /* Bit 5:  Auto Flow Control Enable */
                                               /* Bit 6-7: Reserved */

/* LSR Line Status Register */

#define UART_LSR_DR                  (1 << 0)  /* Bit 0:  Data Ready */
#define UART_LSR_OE                  (1 << 1)  /* Bit 1:  Overrun Error */
#define UART_LSR_PE                  (1 << 2)  /* Bit 2:  Parity Error */
#define UART_LSR_FE                  (1 << 3)  /* Bit 3:  Framing Error */
#define UART_LSR_BI                  (1 << 4)  /* Bit 4:  Break Interrupt */
#define UART_LSR_THRE                (1 << 5)  /* Bit 5:  Transmitter Holding Register Empty */
#define UART_LSR_TEMT                (1 << 6)  /* Bit 6:  Transmitter Empty */
#define UART_LSR_RXFE                (1 << 7)  /* Bit 7:  Error in RX FIFO (RXFE) */

/* SCR Scratch Pad Register */

#define UART_SCR_MASK                (0xff)    /* Bits 0-7: SCR data */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#if CONFIG_16550_REGWIDTH == 8
typedef uint8_t uart_datawidth_t;
#elif CONFIG_16550_REGWIDTH == 16
typedef uint16_t uart_datawidth_t;
#elif CONFIG_16550_REGWIDTH == 32
typedef uint32_t uart_datawidth_t;
#endif

#if CONFIG_16550_ADDRWIDTH == 0
typedef uintptr_t uart_addrwidth_t;
#elif CONFIG_16550_ADDRWIDTH == 8
typedef uint8_t uart_addrwidth_t;
#elif CONFIG_16550_ADDRWIDTH == 16
typedef uint16_t uart_addrwidth_t;
#elif CONFIG_16550_ADDRWIDTH == 32
typedef uint32_t uart_addrwidth_t;
#elif CONFIG_16550_ADDRWIDTH == 64
typedef uint64_t uart_addrwidth_t;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: u16550_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before uart_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by uart_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

void u16550_earlyserialinit(void);

/****************************************************************************
 * Name: u16550_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   u16550_earlyserialinit was called previously.
 *
 ****************************************************************************/

void u16550_serialinit(void);

/****************************************************************************
 * Name: uart_getreg(), uart_putreg(), uart_ioctl()
 *
 * Description:
 *   These functions must be provided by the processor-specific code in order
 *   to correctly access 16550 registers
 *   uart_ioctl() is optional to provide custom IOCTLs
 *
 ****************************************************************************/

#ifndef CONFIG_SERIAL_UART_ARCH_MMIO
uart_datawidth_t uart_getreg(uart_addrwidth_t base, unsigned int offset);
void uart_putreg(uart_addrwidth_t base,
                 unsigned int offset,
                 uart_datawidth_t value);
#endif

struct file;  /* Forward reference */
int uart_ioctl(struct file *filep, int cmd, unsigned long arg);

struct dma_chan_s;
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base,
                                    unsigned int ident);

#endif /* CONFIG_16550_UART */
#endif /* __INCLUDE_NUTTX_SERIAL_UART_16550_H */
