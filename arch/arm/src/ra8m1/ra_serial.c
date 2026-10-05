/****************************************************************************
 * arch/arm/src/ra8m1/ra_serial.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/ra8m1_sci.h"
#include "hardware/ra8m1_mstp.h"
#include "hardware/ra8m1_system.h"
#include "ra_clockconfig.h"
#include "ra_lowputc.h"
#include "ra_icu.h"
#include "ra_gpio.h"

#ifdef CONFIG_RA_DTC
#  include <nuttx/cache.h>

#  include "hardware/ra8m1_dtc.h"
#  include "ra_dtc.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RA8M1 only implements SCI_B0-SCI_B4 and SCI_B9 (SCI_B5-8 do not exist). */

/* Is there a serial console?  */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI0_UART)
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI1_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI2_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI3_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI4_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI9_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#else
#ifndef CONFIG_NO_SERIAL_CONSOLE
#warning "No valid CONFIG_SCIn_SERIAL_CONSOLE Setting"
#endif

#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#undef HAVE_CONSOLE
#endif

/* First pick the console and ttys0. */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart0port /* UART0 is console */
#define TTYS0_DEV       g_uart0port /* UART0 is ttyS0 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart1port /* UART1 is console */
#define TTYS0_DEV       g_uart1port /* UART1 is ttyS0 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart2port /* UART2 is console */
#define TTYS0_DEV       g_uart2port /* UART2 is ttyS0 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart3port /* UART3 is console */
#define TTYS0_DEV       g_uart3port /* UART3 is ttyS0 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart4port /* UART4 is console */
#define TTYS0_DEV       g_uart4port /* UART4 is ttyS0 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart9port /* UART9 is console */
#define TTYS0_DEV       g_uart9port /* UART9 is ttyS0 */
#define UART9_ASSIGNED  1
#else
#undef CONSOLE_DEV                  /* No console */
#if defined(CONFIG_RA_SCI0_UART)
#define TTYS0_DEV       g_uart0port /* UART0 is ttyS0 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_RA_SCI1_UART)
#define TTYS0_DEV       g_uart1port /* UART1 is ttyS0 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_RA_SCI2_UART)
#define TTYS0_DEV       g_uart2port /* UART2 is ttyS0 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_RA_SCI3_UART)
#define TTYS0_DEV       g_uart3port /* UART3 is ttyS0 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_RA_SCI4_UART)
#define TTYS0_DEV       g_uart4port /* UART4 is ttyS0 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_RA_SCI9_UART)
#define TTYS0_DEV       g_uart9port /* UART9 is ttyS0 */
#define UART9_ASSIGNED  1
#endif
#endif

/* Pick ttys1. */

#if defined(CONFIG_RA_SCI0_UART) && !defined(UART0_ASSIGNED)
#define TTYS1_DEV       g_uart0port /* UART0 is ttyS1 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_RA_SCI1_UART) && !defined(UART1_ASSIGNED)
#define TTYS1_DEV       g_uart1port /* UART1 is ttyS1 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_RA_SCI2_UART) && !defined(UART2_ASSIGNED)
#define TTYS1_DEV       g_uart2port /* UART2 is ttyS1 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_RA_SCI3_UART) && !defined(UART3_ASSIGNED)
#define TTYS1_DEV       g_uart3port /* UART3 is ttyS1 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_RA_SCI4_UART) && !defined(UART4_ASSIGNED)
#define TTYS1_DEV       g_uart4port /* UART4 is ttyS1 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_RA_SCI9_UART) && !defined(UART9_ASSIGNED)
#define TTYS1_DEV       g_uart9port /* UART9 is ttyS1 */
#define UART9_ASSIGNED  1
#endif

/* Pick ttys2. */

#if defined(CONFIG_RA_SCI0_UART) && !defined(UART0_ASSIGNED)
#define TTYS2_DEV       g_uart0port /* UART0 is ttyS2 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_RA_SCI1_UART) && !defined(UART1_ASSIGNED)
#define TTYS2_DEV       g_uart1port /* UART1 is ttyS2 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_RA_SCI2_UART) && !defined(UART2_ASSIGNED)
#define TTYS2_DEV       g_uart2port /* UART2 is ttyS2 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_RA_SCI3_UART) && !defined(UART3_ASSIGNED)
#define TTYS2_DEV       g_uart3port /* UART3 is ttyS2 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_RA_SCI4_UART) && !defined(UART4_ASSIGNED)
#define TTYS2_DEV       g_uart4port /* UART4 is ttyS2 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_RA_SCI9_UART) && !defined(UART9_ASSIGNED)
#define TTYS2_DEV       g_uart9port /* UART9 is ttyS2 */
#define UART9_ASSIGNED  1
#endif

/* Pick ttys3. */

#if defined(CONFIG_RA_SCI0_UART) && !defined(UART0_ASSIGNED)
#define TTYS3_DEV       g_uart0port /* UART0 is ttyS3 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_RA_SCI1_UART) && !defined(UART1_ASSIGNED)
#define TTYS3_DEV       g_uart1port /* UART1 is ttyS3 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_RA_SCI2_UART) && !defined(UART2_ASSIGNED)
#define TTYS3_DEV       g_uart2port /* UART2 is ttyS3 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_RA_SCI3_UART) && !defined(UART3_ASSIGNED)
#define TTYS3_DEV       g_uart3port /* UART3 is ttyS3 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_RA_SCI4_UART) && !defined(UART4_ASSIGNED)
#define TTYS3_DEV       g_uart4port /* UART4 is ttyS3 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_RA_SCI9_UART) && !defined(UART9_ASSIGNED)
#define TTYS3_DEV       g_uart9port /* UART9 is ttyS3 */
#define UART9_ASSIGNED  1
#endif

/* Pick ttys4. */

#if defined(CONFIG_RA_SCI0_UART) && !defined(UART0_ASSIGNED)
#define TTYS4_DEV       g_uart0port /* UART0 is ttyS4 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_RA_SCI1_UART) && !defined(UART1_ASSIGNED)
#define TTYS4_DEV       g_uart1port /* UART1 is ttyS4 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_RA_SCI2_UART) && !defined(UART2_ASSIGNED)
#define TTYS4_DEV       g_uart2port /* UART2 is ttyS4 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_RA_SCI3_UART) && !defined(UART3_ASSIGNED)
#define TTYS4_DEV       g_uart3port /* UART3 is ttyS4 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_RA_SCI4_UART) && !defined(UART4_ASSIGNED)
#define TTYS4_DEV       g_uart4port /* UART4 is ttyS4 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_RA_SCI9_UART) && !defined(UART9_ASSIGNED)
#define TTYS4_DEV       g_uart9port /* UART9 is ttyS4 */
#define UART9_ASSIGNED  1
#endif

/* Pick ttys5. */

#if defined(CONFIG_RA_SCI0_UART) && !defined(UART0_ASSIGNED)
#define TTYS5_DEV       g_uart0port /* UART0 is ttyS5 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_RA_SCI1_UART) && !defined(UART1_ASSIGNED)
#define TTYS5_DEV       g_uart1port /* UART1 is ttyS5 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_RA_SCI2_UART) && !defined(UART2_ASSIGNED)
#define TTYS5_DEV       g_uart2port /* UART2 is ttyS5 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_RA_SCI3_UART) && !defined(UART3_ASSIGNED)
#define TTYS5_DEV       g_uart3port /* UART3 is ttyS5 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_RA_SCI4_UART) && !defined(UART4_ASSIGNED)
#define TTYS5_DEV       g_uart4port /* UART4 is ttyS5 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_RA_SCI9_UART) && !defined(UART9_ASSIGNED)
#define TTYS5_DEV       g_uart9port /* UART9 is ttyS5 */
#define UART9_ASSIGNED  1
#endif

#define SCI_UART_ERR_BITS  (R_SCI_B_CSR_PER | R_SCI_B_CSR_FER | \
                             R_SCI_B_CSR_ORER)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Forward-declared so up_dma_send_at()'s prototype below shares the same
 * file-scope tag as the real definition further down, instead of GCC
 * implicitly declaring a distinct, incompatible one scoped to the
 * prototype's parameter list.
 */

struct up_dev_s;

static int up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int up_rxinterrupt(int irq, void *context, void *arg);
static int up_txinterrupt(int irq, void *context, void *arg);
static int up_erinterrupt(int irq, void *context, void *arg);
static int up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_TXDMA
static void up_dma_send_at(struct up_dev_s *priv, uintptr_t buffer,
                            size_t length);
static void up_dma_send(struct uart_dev_s *dev);
static void up_dma_txavail(struct uart_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct up_dev_s
{
  const uint32_t scibase;   /* Base address of SCI_B registers */
  uint32_t mstp;            /* Module Stop Control Register bit */
  uint32_t baud;            /* Configured baud */
  uint32_t sr;              /* Saved status bits */
  uint8_t rxirq;            /* IRQ associated with this SCI */
  uint8_t txirq;            /* IRQ associated with this SCI */
  uint8_t teirq;            /* IRQ associated with this SCI */
  uint8_t erirq;            /* IRQ associated with this SCI */
  uint8_t parity;           /* 0=none, 1=odd, 2=even */
  uint8_t bits;             /* Number of bits (7, 8 or 9) */
  bool stopbits2;           /* true: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_RA_SCI_FIFO
  bool fifo;                /* true: 16-stage FIFO mode (CCR3.FM), not 1-stage */
  uint8_t ttrg;             /* FCR.TTRG when priv->fifo (0-15) */
  uint8_t rtrg;             /* FCR.RTRG when priv->fifo (0-15) */
#endif
#ifdef CONFIG_RA_DTC
  bool txdtc;               /* true: transmit through the DTC, not per-byte */
  bool rxdtc;               /* true: receive through the DTC, not per-byte */
  bool rxdtc_pending;       /* true: dtcrxbyte holds an unread DTC'd byte */
  uint8_t dtcrxbyte;        /* Scratch destination for the RX DTC transfer */

  /* This port's DTC transfer info: one for TX, one for RX -- both may be
   * armed at once, on different IELSR vectors, so they cannot share one
   * block.
   */

  struct dtc_transfer_info_s dtcinfo;
  struct dtc_transfer_info_s dtcrxinfo;
#endif
#ifdef CONFIG_SERIAL_TIOCGICOUNT
  struct serial_icounter_s icount;  /* Frame/overrun/parity error counts */
#endif
};

#ifdef CONFIG_RA_DTC
static void up_dma_rxarm(struct up_dev_s *priv);
#endif

static const struct uart_ops_s g_uart_ops =
{
  .setup        = up_setup,
  .shutdown     = up_shutdown,
  .attach       = up_attach,
  .detach       = up_detach,
  .ioctl        = up_ioctl,
  .receive      = up_receive,
  .rxint        = up_rxint,
  .rxavailable  = up_rxavailable,
  .send         = up_send,
  .txint        = up_txint,
  .txready      = up_txready,
  .txempty      = up_txempty,
#ifdef CONFIG_SERIAL_TXDMA
  .dmasend      = up_dma_send,
  .dmatxavail   = up_dma_txavail,
#endif
};

/* I/O buffers */

#if defined(CONFIG_RA_SCI0_UART)
static char g_uart0rxbuffer[CONFIG_SCI0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_SCI0_TXBUFSIZE];
#endif
#if defined(CONFIG_RA_SCI1_UART)
static char g_uart1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif
#if defined(CONFIG_RA_SCI2_UART)
static char g_uart2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_SCI2_TXBUFSIZE];
#endif
#if defined(CONFIG_RA_SCI3_UART)
static char g_uart3rxbuffer[CONFIG_SCI3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_SCI3_TXBUFSIZE];
#endif
#if defined(CONFIG_RA_SCI4_UART)
static char g_uart4rxbuffer[CONFIG_SCI4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_SCI4_TXBUFSIZE];
#endif
#if defined(CONFIG_RA_SCI9_UART)
static char g_uart9rxbuffer[CONFIG_SCI9_RXBUFSIZE];
static char g_uart9txbuffer[CONFIG_SCI9_TXBUFSIZE];
#endif

#if defined(CONFIG_RA_SCI0_UART)
static struct up_dev_s  g_uart0priv =
{
  .scibase      = R_SCI_B0_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB31,
  .rxirq        = SCI0_RXI,
  .txirq        = SCI0_TXI,
  .teirq        = SCI0_TEI,
  .erirq        = SCI0_ERI,
  .baud         = CONFIG_SCI0_BAUD,
  .parity       = CONFIG_SCI0_PARITY,
  .bits         = CONFIG_SCI0_BITS,
  .stopbits2    = CONFIG_SCI0_2STOP,
#ifdef CONFIG_RA_SCI0_FIFO
  .fifo         = true,
  .ttrg         = CONFIG_RA_SCI0_FIFO_TXTRG,
  .rtrg         = CONFIG_RA_SCI0_FIFO_RXTRG,
#endif
#ifdef CONFIG_RA_SCI0_TXDTC
  .txdtc        = true,
#endif
#ifdef CONFIG_RA_SCI0_RXDTC
  .rxdtc        = true,
#endif
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_SCI0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart0priv,
};
#endif

#if defined(CONFIG_RA_SCI1_UART)
static struct up_dev_s  g_uart1priv =
{
  .scibase      = R_SCI_B1_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB30,
  .rxirq        = SCI1_RXI,
  .txirq        = SCI1_TXI,
  .teirq        = SCI1_TEI,
  .erirq        = SCI1_ERI,
  .baud         = CONFIG_SCI1_BAUD,
  .parity       = CONFIG_SCI1_PARITY,
  .bits         = CONFIG_SCI1_BITS,
  .stopbits2    = CONFIG_SCI1_2STOP,
#ifdef CONFIG_RA_SCI1_FIFO
  .fifo         = true,
  .ttrg         = CONFIG_RA_SCI1_FIFO_TXTRG,
  .rtrg         = CONFIG_RA_SCI1_FIFO_RXTRG,
#endif
#ifdef CONFIG_RA_SCI1_TXDTC
  .txdtc        = true,
#endif
#ifdef CONFIG_RA_SCI1_RXDTC
  .rxdtc        = true,
#endif
};

static uart_dev_t  g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_SCI1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart1priv,
};
#endif

#if defined(CONFIG_RA_SCI2_UART)
static struct up_dev_s  g_uart2priv =
{
  .scibase      = R_SCI_B2_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB29,
  .rxirq        = SCI2_RXI,
  .txirq        = SCI2_TXI,
  .teirq        = SCI2_TEI,
  .erirq        = SCI2_ERI,
  .baud         = CONFIG_SCI2_BAUD,
  .parity       = CONFIG_SCI2_PARITY,
  .bits         = CONFIG_SCI2_BITS,
  .stopbits2    = CONFIG_SCI2_2STOP,
#ifdef CONFIG_RA_SCI2_FIFO
  .fifo         = true,
  .ttrg         = CONFIG_RA_SCI2_FIFO_TXTRG,
  .rtrg         = CONFIG_RA_SCI2_FIFO_RXTRG,
#endif
#ifdef CONFIG_RA_SCI2_TXDTC
  .txdtc        = true,
#endif
#ifdef CONFIG_RA_SCI2_RXDTC
  .rxdtc        = true,
#endif
};

static uart_dev_t  g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_SCI2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart2priv,
};
#endif

#if defined(CONFIG_RA_SCI3_UART)
static struct up_dev_s  g_uart3priv =
{
  .scibase      = R_SCI_B3_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB28,
  .rxirq        = SCI3_RXI,
  .txirq        = SCI3_TXI,
  .teirq        = SCI3_TEI,
  .erirq        = SCI3_ERI,
  .baud         = CONFIG_SCI3_BAUD,
  .parity       = CONFIG_SCI3_PARITY,
  .bits         = CONFIG_SCI3_BITS,
  .stopbits2    = CONFIG_SCI3_2STOP,
#ifdef CONFIG_RA_SCI3_FIFO
  .fifo         = true,
  .ttrg         = CONFIG_RA_SCI3_FIFO_TXTRG,
  .rtrg         = CONFIG_RA_SCI3_FIFO_RXTRG,
#endif
#ifdef CONFIG_RA_SCI3_TXDTC
  .txdtc        = true,
#endif
#ifdef CONFIG_RA_SCI3_RXDTC
  .rxdtc        = true,
#endif
};

static uart_dev_t  g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_SCI3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart3priv,
};
#endif

#if defined(CONFIG_RA_SCI4_UART)
static struct up_dev_s  g_uart4priv =
{
  .scibase      = R_SCI_B4_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB27,
  .rxirq        = SCI4_RXI,
  .txirq        = SCI4_TXI,
  .teirq        = SCI4_TEI,
  .erirq        = SCI4_ERI,
  .baud         = CONFIG_SCI4_BAUD,
  .parity       = CONFIG_SCI4_PARITY,
  .bits         = CONFIG_SCI4_BITS,
  .stopbits2    = CONFIG_SCI4_2STOP,
#ifdef CONFIG_RA_SCI4_FIFO
  .fifo         = true,
  .ttrg         = CONFIG_RA_SCI4_FIFO_TXTRG,
  .rtrg         = CONFIG_RA_SCI4_FIFO_RXTRG,
#endif
#ifdef CONFIG_RA_SCI4_TXDTC
  .txdtc        = true,
#endif
#ifdef CONFIG_RA_SCI4_RXDTC
  .rxdtc        = true,
#endif
};

static uart_dev_t  g_uart4port =
{
  .recv     =
  {
    .size   = CONFIG_SCI4_RXBUFSIZE,
    .buffer = g_uart4rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI4_TXBUFSIZE,
    .buffer = g_uart4txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart4priv,
};
#endif

#if defined(CONFIG_RA_SCI9_UART)
static struct up_dev_s  g_uart9priv =
{
  .scibase      = R_SCI_B9_BASE,
  .mstp         = R_MSTP_MSTPCRB_MSTPB22,
  .rxirq        = SCI9_RXI,
  .txirq        = SCI9_TXI,
  .teirq        = SCI9_TEI,
  .erirq        = SCI9_ERI,
  .baud         = CONFIG_SCI9_BAUD,
  .parity       = CONFIG_SCI9_PARITY,
  .bits         = CONFIG_SCI9_BITS,
  .stopbits2    = CONFIG_SCI9_2STOP,
#ifdef CONFIG_RA_SCI9_FIFO
  .fifo         = true,
  .ttrg         = CONFIG_RA_SCI9_FIFO_TXTRG,
  .rtrg         = CONFIG_RA_SCI9_FIFO_RXTRG,
#endif
#ifdef CONFIG_RA_SCI9_TXDTC
  .txdtc        = true,
#endif
#ifdef CONFIG_RA_SCI9_RXDTC
  .rxdtc        = true,
#endif
};

static uart_dev_t  g_uart9port =
{
  .recv     =
  {
    .size   = CONFIG_SCI9_RXBUFSIZE,
    .buffer = g_uart9rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI9_TXBUFSIZE,
    .buffer = g_uart9txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart9priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->scibase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                 uint32_t value)
{
  putreg32(value, priv->scibase + offset);
}

/****************************************************************************
 * Name: up_disableallints
 ****************************************************************************/

static void up_disableallints(struct up_dev_s *priv, uint32_t *ie)
{
  irqstate_t flags;
  uint32_t  regval = 0;

  /* The following must be atomic */

  flags = enter_critical_section();
  if (ie)
    {
      /* Return the current interrupt mask */

      *ie = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
    }

  /* Disable all interrupts */

  regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET) &
    ~(R_SCI_B_CCR0_TIE | R_SCI_B_CCR0_RIE);
  up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_sci_config
 *
 * Description:
 *   Configure the SCI_B baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static void up_sci_config(struct up_dev_s *priv)
{
  uint32_t  regval          = 0;

  /* Disable the channel while it is (re)configured */

  up_serialout(priv, R_SCI_B_CCR0_OFFSET, 0);

  /* Character length and stop bits live in CCR3.CHR/CCR3.STP.  Unlike the
   * legacy SCI, SCI_B has no separate SCMR.CHR1 -- CCR3.CHR is a single
   * 2-bit field:  00/01 = 9-bit, 10 = 8-bit (reset value), 11 = 7-bit.
   */

  regval  = up_serialin(priv, R_SCI_B_CCR3_OFFSET);
  regval &= ~(R_SCI_B_CCR3_CHR_MASK << R_SCI_B_CCR3_CHR_SHIFT);

  if (priv->bits == 9)
    {
      regval |= R_SCI_B_CCR3_CHR_V00;
    }
  else if (priv->bits == 7)
    {
      regval |= R_SCI_B_CCR3_CHR_V11;
    }
  else
    {
      regval |= R_SCI_B_CCR3_CHR_V10;
    }

  if (priv->stopbits2)
    {
      regval |= R_SCI_B_CCR3_STP;
    }
  else
    {
      regval &= ~R_SCI_B_CCR3_STP;
    }

  regval &= ~(R_SCI_B_CCR3_MOD_MASK << R_SCI_B_CCR3_MOD_SHIFT);
  regval |= R_SCI_B_CCR3_MOD_ASYNCHRONOUS;

#ifdef CONFIG_RA_SCI_FIFO
  if (priv->fifo)
    {
      regval |= R_SCI_B_CCR3_FM;
    }
  else
#endif
    {
      regval &= ~R_SCI_B_CCR3_FM;
    }

  up_serialout(priv, R_SCI_B_CCR3_OFFSET, regval);

  /* Parity lives in CCR1.PE/CCR1.PM */

  regval  = up_serialin(priv, R_SCI_B_CCR1_OFFSET);
  regval &= ~(R_SCI_B_CCR1_PE | R_SCI_B_CCR1_PM);

  if (priv->parity > 0)
    {
      regval |= R_SCI_B_CCR1_PE;
      if (priv->parity == 1)
        {
          regval |= R_SCI_B_CCR1_PM;
        }
    }

  up_serialout(priv, R_SCI_B_CCR1_OFFSET, regval);

  /* The baud rate generator lives in CCR2 (BGDM/ABCS/ABCSE, CKS and BRR).
   * Its clock is SCICLK, not PCLKA, as long as the synchronizer bypass
   * (CCR3.BPEN) stays off.  ra_sci_baud_ccr2() picks the settings with the
   * smallest error (RA8M1 User's Manual Table 31.7).
   */

  regval  = up_serialin(priv, R_SCI_B_CCR2_OFFSET);
  regval &= ~(R_SCI_B_CCR2_BGDM | R_SCI_B_CCR2_ABCS | R_SCI_B_CCR2_ABCSE |
              (R_SCI_B_CCR2_BRR_MASK << R_SCI_B_CCR2_BRR_SHIFT) |
              (R_SCI_B_CCR2_CKS_MASK << R_SCI_B_CCR2_CKS_SHIFT));
  regval |= ra_sci_baud_ccr2(RA_SCICLK_FREQUENCY, priv->baud);
  up_serialout(priv, R_SCI_B_CCR2_OFFSET, regval);

#ifdef CONFIG_RA_SCI_FIFO
  if (priv->fifo)
    {
      /* RA8M1 User's Manual Table 31.30, step 7: empty both FIFOs
       * (TFRST/RFRST are write-1, self-clearing) and set the trigger
       * levels in the same write.  DRES=0 routes the receive-timeout
       * (idle-line) event to SCIn_RXI, same as a threshold hit, so
       * up_rxinterrupt() needs only one code path.  RSTRG is left at 0
       * (no hardware flow control).
       *
       * Must come after the CCR3 write that sets CCR3.FM: FCR silently
       * fails to latch RTRG/TTRG if written too soon after CCR3 (needs
       * the settling time the CCR2/CCR1 writes above provide anyway --
       * matches Renesas FSP's r_sci_b_uart_config_set() ordering).
       */

      regval = R_SCI_B_FCR_TFRST | R_SCI_B_FCR_RFRST |
               ((uint32_t)priv->ttrg << R_SCI_B_FCR_TTRG_SHIFT) |
               ((uint32_t)priv->rtrg << R_SCI_B_FCR_RTRG_SHIFT);
      up_serialout(priv, R_SCI_B_FCR_OFFSET, regval);

      /* Table 31.30, step 11: drop any stale DR/BRK flags. */

      up_serialout(priv, R_SCI_B_FFCLR_OFFSET, R_SCI_B_FFCLR_DRC);

      /* CSR.TDRE/RDRF are the real SCIn_TXI/SCIn_RXI sources in FIFO
       * mode and do not auto-clear; clear them through CFCLR here so
       * the stale TDRE=1 left by TFRST above doesn't swallow the first
       * 0->1 edge once up_send()'s initial fake-kick burst drains.
       */

      up_serialout(priv, R_SCI_B_CFCLR_OFFSET,
                   R_SCI_B_CFCLR_TDREC | R_SCI_B_CFCLR_RDRFC);
    }
#endif

  /* Re-enable the channel: transmit/receive plus their interrupts */

  regval = (R_SCI_B_CCR0_TE | R_SCI_B_CCR0_RE | R_SCI_B_CCR0_TIE |
            R_SCI_B_CCR0_RIE | R_SCI_B_CCR0_TEIE);
  up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);
}

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Dispatch on the channel's own base address rather than a global
   * #elif: several SCI_B channels may be enabled at once (see the
   * ttyS0-ttyS5 assignment ladder above), so GPIO setup must be chosen
   * per-device, not per-build.
   */

#if defined(CONFIG_RA_SCI0_UART)
  if (priv->scibase == R_SCI_B0_BASE)
    {
      ra_configgpio(GPIO_SCI0_RX);
      ra_configgpio(GPIO_SCI0_TX);
    }

#endif
#if defined(CONFIG_RA_SCI1_UART)
  if (priv->scibase == R_SCI_B1_BASE)
    {
      ra_configgpio(GPIO_SCI1_RX);
      ra_configgpio(GPIO_SCI1_TX);
    }

#endif
#if defined(CONFIG_RA_SCI2_UART)
  if (priv->scibase == R_SCI_B2_BASE)
    {
      ra_configgpio(GPIO_SCI2_RX);
      ra_configgpio(GPIO_SCI2_TX);
    }

#endif
#if defined(CONFIG_RA_SCI3_UART)
  if (priv->scibase == R_SCI_B3_BASE)
    {
      ra_configgpio(GPIO_SCI3_RX);
      ra_configgpio(GPIO_SCI3_TX);
    }

#endif
#if defined(CONFIG_RA_SCI4_UART)
  if (priv->scibase == R_SCI_B4_BASE)
    {
      ra_configgpio(GPIO_SCI4_RX);
      ra_configgpio(GPIO_SCI4_TX);
    }

#endif
#if defined(CONFIG_RA_SCI9_UART)
  if (priv->scibase == R_SCI_B9_BASE)
    {
      ra_configgpio(GPIO_SCI9_RX);
      ra_configgpio(GPIO_SCI9_TX);
    }

#endif

  up_shutdown(dev);

  putreg16((R_SYSTEM_PRCR_S_PRKEY_V0XA5 | R_SYSTEM_PRCR_S_PRC1),
           R_SYSTEM_PRCR_S);
  modifyreg32(R_MSTP_MSTPCRB, priv->mstp, 0);
  putreg16(R_SYSTEM_PRCR_S_PRKEY_V0XA5, R_SYSTEM_PRCR_S);

  up_sci_config(priv);

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the SCI_B channel.
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Reset SCI_B control */

  up_serialout(priv, R_SCI_B_CCR0_OFFSET, 0);

  /* Stop the channel's module clock */

  putreg16((R_SYSTEM_PRCR_S_PRKEY_V0XA5 | R_SYSTEM_PRCR_S_PRC1),
           R_SYSTEM_PRCR_S);
  modifyreg32(R_MSTP_MSTPCRB, priv->mstp, 1);
  putreg16(R_SYSTEM_PRCR_S_PRKEY_V0XA5, R_SYSTEM_PRCR_S);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the SCI to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s   *priv = (struct up_dev_s *)dev->priv;
  int               ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->rxirq, up_rxinterrupt, dev);
  if (ret < 0)
    {
      return ret;
    }

  ret = irq_attach(priv->txirq, up_txinterrupt, dev);
  if (ret < 0)
    {
      irq_detach(priv->rxirq);
      return ret;
    }

  ret = irq_attach(priv->erirq, up_erinterrupt, dev);
  if (ret < 0)
    {
      irq_detach(priv->erirq);
      return ret;
    }

  up_enable_irq(priv->rxirq);
  up_enable_irq(priv->txirq);
  up_enable_irq(priv->erirq);

#ifdef CONFIG_RA_DTC
  if (priv->rxdtc)
    {
      /* One-time initial arm, before anything can be received.
       * ra_dtc_initialize() has already run by now (from
       * up_irqinitialize(), before any port's up_attach()), so the
       * vector table this points into is valid.  up_rxint() never
       * arms it (see there); up_rxdrain() does on every later drain.
       */

      priv->rxdtc_pending = false;
      up_dma_rxarm(priv);
    }
#endif

  return ret;
}

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  up_disable_irq(priv->rxirq);
  up_disable_irq(priv->txirq);
  up_disable_irq(priv->erirq);
  irq_detach(priv->rxirq);
  irq_detach(priv->txirq);
  irq_detach(priv->erirq);
}

/****************************************************************************
 * Name: up_rxdrain
 *
 * Description:
 *   Deliver whatever is currently available to the upper half and leave
 *   the receive path ready for more.  Shared by up_rxinterrupt() (a
 *   genuine SCIn_RXI) and up_erinterrupt() (SCIn_ERI, which fires instead
 *   of SCIn_RXI for every receive error -- RA8M1 User's Manual section
 *   31.3.9 -- so without this, data already sitting in RDR/the FIFO when
 *   an error occurs is never picked up at all).
 *
 ****************************************************************************/

static void up_rxdrain(struct up_dev_s *priv, struct uart_dev_s *dev)
{
#ifdef CONFIG_RA_DTC
  if (priv->rxdtc && !priv->rxdtc_pending)
    {
      uint8_t vector = priv->rxirq - RA_IRQ_FIRST;

      if ((getreg32(R_ICU_IELSR(vector)) & R_ICU_IELSR_DTCE) == 0)
        {
          /* DTCE clear means the DTC already moved a byte into
           * dtcrxbyte, whether this drain was triggered by that
           * byte's own SCIn_RXI or by a later SCIn_ERI that happened
           * to be serviced first.  Claimed exactly once, here, so the
           * other caller can't also claim (and so double-deliver) it.
           */

          priv->rxdtc_pending = true;
        }
    }
#endif

  uart_recvchars(dev);

#ifdef CONFIG_RA_DTC
  if (priv->rxdtc)
    {
      /* Safe to call unconditionally: up_dma_rxarm() no-ops if DTCE is
       * already 1.
       */

      up_dma_rxarm(priv);
    }
#endif

#ifdef CONFIG_RA_SCI_FIFO
  if (priv->fifo)
    {
      /* uart_recvchars() above just drained the FIFO to FRSR.R == 0.
       * Clearing FRSR.DR unconditionally here is safe (it stays set
       * until both the drain and this write happen -- RA8M1 User's
       * Manual section 31.2.19) and harmless if DR wasn't the cause.
       */

      up_serialout(priv, R_SCI_B_FFCLR_OFFSET, R_SCI_B_FFCLR_DRC);

      /* CSR.RDRF is the real SCIn_RXI source in FIFO mode too (FRSR.R
       * is just a byte counter) and reading RDR doesn't clear it the
       * way it does in non-FIFO mode -- only CFCLR does (section
       * 31.2.9; matches Renesas FSP's sci_b_uart_rxi_isr()).  Leaving
       * it set would latch out the next 0->1 edge and SCIn_RXI would
       * never fire again.
       */

      up_serialout(priv, R_SCI_B_CFCLR_OFFSET, R_SCI_B_CFCLR_RDRFC);
    }
#endif
}

/****************************************************************************
 * Name: up_rxinterrupt
 *
 * Description:
 *   This is the common SCI RX interrupt handler.
 *
 ****************************************************************************/

static int up_rxinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev  = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv = (struct up_dev_s *)dev->priv;

  ra_clear_ir(irq);

  up_rxdrain(priv, dev);

  return OK;
}

/****************************************************************************
 * Name: up_txinterrupt
 *
 * Description:
 *   This is the common SCI TX interrupt handler.
 *
 ****************************************************************************/

static int up_txinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
#if defined(CONFIG_SERIAL_TXDMA) || defined(CONFIG_RA_SCI_FIFO)
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
#endif

  ra_clear_ir(irq);

#ifdef CONFIG_RA_SCI_FIFO
  if (priv->fifo)
    {
      /* CSR.TDRE is the real SCIn_TXI source in FIFO mode too (FTSR.T is
       * just a byte counter) and doesn't auto-clear -- only CFCLR does
       * (section 31.2.9).  Must run BEFORE the fill below: clearing
       * after could erase an edge the fill itself just caused (e.g. at
       * TTRG=15, one byte refills the FIFO straight back past the
       * threshold).  Clearing first loses nothing, since any later drop
       * below TTRG still latches TDRE fresh.
       */

      up_serialout(priv, R_SCI_B_CFCLR_OFFSET, R_SCI_B_CFCLR_TDREC);
    }
#endif

#ifdef CONFIG_SERIAL_TXDMA
  if (priv->txdtc)
    {
      /* dev->dmatx.length is 0 the first time TXI fires after enabling
       * TIE (up_txint()/up_dma_txavail() already tried
       * uart_xmitchars_dma() directly), and the length last armed
       * otherwise -- DTC normal mode has moved every byte of it by now.
       */

      bool chained = false;

      if (dev->dmatx.length != 0)
        {
          dev->dmatx.nbytes += dev->dmatx.length;

          if (dev->dmatx.nlength != 0)
            {
              /* uart_xmitchars_dma() split a wrapped ring buffer into
               * this (now done) segment and a second one at the start
               * of the buffer (dmatx.nbuffer/nlength).  Chain into it
               * here, exactly once -- without this, a wrapping transfer
               * silently drops everything past the wrap.
               *
               * dev->dmatx.buffer is deliberately left pointing at this
               * segment, not the second one: uart_xmitchars_done() below
               * compares it against dev->xmit.tail to decide whether to
               * advance it (serial_dma.c), and that must stay true for
               * BOTH segments until done() finally runs for them
               * together.  dmatx.length is updated so the next
               * completion's nbytes accounting matches this segment,
               * not the first one again.
               */

              FAR char *nbuffer = dev->dmatx.nbuffer;
              size_t nlength = dev->dmatx.nlength;

              dev->dmatx.length  = nlength;
              dev->dmatx.nbuffer = NULL;
              dev->dmatx.nlength = 0;

              up_dma_send_at(priv, (uintptr_t)nbuffer, nlength);
              chained = true;
            }
          else
            {
              uart_xmitchars_done(dev);
            }
        }

      if (!chained)
        {
          uart_xmitchars_dma(dev);

          if (dev->dmatx.length == 0)
            {
              /* Nothing armed: the ring buffer was empty.  up_txint()
               * never touches CCR0.TIE for a txdtc port (see there), so
               * this is the only place that clears it once the last
               * transfer drains; up_dma_send_at() is the only place
               * that sets it again.
               */

              uint32_t regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET);

              regval &= ~R_SCI_B_CCR0_TIE;
              up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);
            }
        }
    }
  else
#endif
    {
      uart_xmitchars(dev);
    }

#ifdef CONFIG_RA_SCI_FIFO
  if (priv->fifo)
    {
      if (dev->xmit.head == dev->xmit.tail)
        {
          /* Nothing left to send, so TIE serves no purpose: disable
           * it here rather than leave it armed for no reason.
           * up_txint(true) re-enables it the next time there's
           * something to send.
           */

          uint32_t regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET);

          regval &= ~R_SCI_B_CCR0_TIE;
          up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: up_erinterrupt
 *
 * Description:
 *   This is the common SCI Error interrupt handler.
 *
 ****************************************************************************/

static int up_erinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint32_t regval = 0;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* IELSRn.IR must be cleared by software or the NVIC re-enters this
   * handler indefinitely (RA8M1 User's Manual section 13.5.1).
   */

  ra_clear_ir(irq);

  /* Save for error reporting */

  priv->sr = up_serialin(priv, R_SCI_B_CSR_OFFSET) & SCI_UART_ERR_BITS;

#ifdef CONFIG_SERIAL_TIOCGICOUNT
  if (priv->sr & R_SCI_B_CSR_ORER)
    {
      priv->icount.overrun++;
    }

  if (priv->sr & R_SCI_B_CSR_PER)
    {
      priv->icount.parity++;
    }

  if (priv->sr & R_SCI_B_CSR_FER)
    {
      priv->icount.frame++;
    }
#endif

  /* Receive errors report only through SCIn_ERI, never SCIn_RXI (RA8M1
   * User's Manual section 31.3.9): drain whatever is already in RDR/the
   * FIFO now, through the same path up_rxinterrupt() uses, or it is
   * never picked up and blocks all further reception -- reception
   * cannot resume while CSR.ORER stays set, and clearing it below does
   * not by itself free up a byte still sitting unread in RDR/the FIFO.
   */

  up_rxdrain(priv, dev);

  /* Errors are cleared through the dedicated CFCLR register, not by
   * writing CSR directly.
   */

  regval = R_SCI_B_CFCLR_ORERC | R_SCI_B_CFCLR_PERC |
           R_SCI_B_CFCLR_FERC | R_SCI_B_CFCLR_ERSC;
  up_serialout(priv, R_SCI_B_CFCLR_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = -ENOTTY;

#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCGICOUNT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;
#endif

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct termios *termiosp = (struct termios *)(uintptr_t)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* Word length (priv->bits) is fixed by CONFIG_SCIn_BITS;
           * TCSETS rejects changing it.  9-bit has no POSIX CSIZE
           * encoding, so report CS8 for it rather than leave this
           * meaningless.
           */

          termiosp->c_cflag = (priv->bits == 7 ? CS7 : CS8) |
                               (priv->parity != 0 ? PARENB : 0) |
                               (priv->parity == 1 ? PARODD : 0) |
                               (priv->stopbits2 ? CSTOPB : 0);

          cfsetispeed(termiosp, priv->baud);

          ret = OK;
        }
        break;

      case TCSETS:
        {
          struct termios *termiosp = (struct termios *)(uintptr_t)arg;
          uint8_t         want_bits;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* Reject a word-length change; FIFO/DTC setup is fixed at
           * build time and not revisited here either.
           */

          want_bits = (termiosp->c_cflag & CSIZE) == CS7 ? 7 : 8;
          if (want_bits != priv->bits)
            {
              ret = -EINVAL;
              break;
            }

          if (termiosp->c_cflag & PARENB)
            {
              priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              priv->parity = 0;
            }

          priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
          priv->baud      = cfgetispeed(termiosp);

          /* Apply immediately (TCSADRAIN/TCSAFLUSH not implemented,
           * same as every other NuttX serial driver's TCSETS); this
           * briefly disables CCR0.TE/RE, so a transfer in flight can
           * be truncated.
           */

          up_sci_config(priv);

          ret = OK;
        }
        break;
#endif

#ifdef CONFIG_SERIAL_TIOCGICOUNT
      case TIOCGICOUNT:
        {
          struct serial_icounter_s *icount =
            (struct serial_icounter_s *)(uintptr_t)arg;

          if (!icount)
            {
              ret = -EINVAL;
              break;
            }

          memcpy(icount, &priv->icount, sizeof(struct serial_icounter_s));
          ret = OK;
        }
        break;
#endif

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return the error information in the saved status */

  *status   = priv->sr;
  priv->sr  = 0;

#ifdef CONFIG_RA_DTC
  if (priv->rxdtc)
    {
      if (!priv->rxdtc_pending)
        {
          /* Reached only via the CSR.RDRF fallback in up_rxavailable():
           * the DTC never actually moved this byte.  Read it directly.
           */

          return (int)(up_serialin(priv, R_SCI_B_RDR_OFFSET) &
                       R_SCI_B_RDR_RDAT_MASK & 0xff);
        }

      /* The DTC writes this byte directly to SRAM, bypassing the CPU
       * cache: invalidate before reading it back.  Only safe as-is
       * while CONFIG_ARMV8M_DCACHE is unset (true for every config
       * tested so far) -- dtcrxbyte shares a cache line with CPU-
       * written fields (e.g. rxdtc_pending), so invalidating it would
       * also discard those writes (TODO: give dtcrxbyte its own cache
       * line before enabling dcache here).
       */

      uintptr_t byteaddr = (uintptr_t)&priv->dtcrxbyte;

      up_invalidate_dcache(byteaddr, byteaddr + sizeof(priv->dtcrxbyte));
      priv->rxdtc_pending = false;

      return (int)priv->dtcrxbyte;
    }
#endif

  /* Then return the actual received byte.  RDR.RDAT is 9 bits wide, but
   * only the low 8 matter for byte-oriented (7/8-bit) transfers.
   */

  return (int)(up_serialin(priv, R_SCI_B_RDR_OFFSET) &
               R_SCI_B_RDR_RDAT_MASK & 0xff);
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t regval;

  irqstate_t flags;

  flags = enter_critical_section();

#ifdef CONFIG_RA_DTC
  if (priv->rxdtc)
    {
      /* uart_readv()'s blocking-read path toggles this on every read
       * that finds the ring buffer empty, purely to make that check
       * atomic -- not because it wants reception actually stopped.  A
       * complete no-op here: the DTC stays armed, and RIE/DTCE/
       * rxdtc_pending are left exactly as up_attach()/up_rxdrain()
       * maintain them.  (Previously this disarmed/
       * re-armed the DTC on every toggle, which could drop a byte
       * in flight mid-toggle -- confirmed on hardware as pasted input
       * getting stuck after one byte.)
       */

      leave_critical_section(flags);
      return;
    }
#endif

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the RX interrupt */

      regval  = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval |= (R_SCI_B_CCR0_RIE);
      up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);
#endif
    }
  else
    {
      /* Disable the RX interrupt */

      regval  = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval &= ~(R_SCI_B_CCR0_RIE);
      up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

#ifdef CONFIG_RA_DTC
  if (priv->rxdtc)
    {
      /* rxdtc_pending covers the normal case (DTC already moved the
       * byte).  CSR.RDRF covers a byte the DTC never touched at all,
       * e.g. one whose own event was ignored because IR was already 1
       * from the byte before it (RA8M1 User's Manual Table 13.5, note 2).
       */

      return priv->rxdtc_pending ||
             (up_serialin(priv, R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_RDRF) != 0;
    }
#endif

#ifdef CONFIG_RA_SCI_FIFO
  if (priv->fifo)
    {
      /* CSR.RDRF does not apply in FIFO mode; FRSR.R is the receive-FIFO
       * byte count.  This also covers the receive-timeout ("idle line")
       * case: FRSR.DR only ever sets when FRSR.R is already nonzero (the
       * whole point of the timeout is that fewer bytes than the trigger
       * level arrived), so checking R here answers both causes at once.
       */

      return ((up_serialin(priv, R_SCI_B_FRSR_OFFSET) >>
               R_SCI_B_FRSR_R_SHIFT) & R_SCI_B_FRSR_R_MASK) != 0;
    }
#endif

  return (up_serialin(priv,
                         R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_RDRF) ==
         R_SCI_B_CSR_RDRF;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  up_serialout(priv, R_SCI_B_TDR_OFFSET,
               (uint32_t)ch & R_SCI_B_TDR_TDAT_MASK);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t regval;

  irqstate_t flags;

#ifdef CONFIG_SERIAL_TXDMA
  if (priv->txdtc)
    {
      /* CCR0.TIE is not a generic interrupt mask for a txdtc port: in
       * non-FIFO mode, TIE=0 stops the SCI from generating SCIn_TXI at
       * all (section 31.12.2(1)), and the DTC's activation (IELSRn.
       * DTCE) feeds off that same event.  The generic serial core calls
       * uart_disabletxint() assuming disabling is always harmless, but
       * here it can permanently silence a DTC transfer still waiting on
       * a future TDR->TSR edge (confirmed on hardware: hung nsh's
       * multi-write output partway through).
       *
       * So enable only attempts a DMA kick (up_dma_txavail() no-ops if
       * a transfer is already in flight) and disable does nothing.
       * up_dma_send_at() is the only place that sets TIE; the idle
       * branch of up_txinterrupt() is the only place that clears it.
       */

      if (enable)
        {
          up_dma_txavail(dev);
        }

      return;
    }
#endif

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the TX interrupt */

      regval  = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval |= (R_SCI_B_CCR0_TIE);
      up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

#ifdef CONFIG_RA_SCI_FIFO
      if (priv->fifo)
        {
          /* Clear any stale TDRE latched while TIE was off before this
           * fake kick fills the FIFO, not after -- same reasoning as
           * up_txinterrupt()'s TDREC clear.
           */

          up_serialout(priv, R_SCI_B_CFCLR_OFFSET, R_SCI_B_CFCLR_TDREC);
        }

#endif
      uart_xmitchars(dev);

#endif
    }
  else
    {
      /* Disable the TX interrupt */

      regval  = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval &= ~(R_SCI_B_CCR0_TIE);
      up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (CSR.TDRE)
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s   *priv   = (struct up_dev_s *)dev->priv;
  bool              ret;

#ifdef CONFIG_RA_SCI_FIFO
  if (priv->fifo)
    {
      /* CSR.TDRE means "count at or below TTRG" in FIFO mode, a
       * threshold, not "any room at all": use the transmit-FIFO count
       * against its 16-entry depth instead.
       */

      uint32_t used = (up_serialin(priv, R_SCI_B_FTSR_OFFSET) >>
                       R_SCI_B_FTSR_T_SHIFT) & R_SCI_B_FTSR_T_MASK;

      return used < 16;
    }
#endif

  ret = ((up_serialin(priv,
                     R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_TDRE) ==
     R_SCI_B_CSR_TDRE);

  return ret;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s   *priv   = (struct up_dev_s *)dev->priv;
  bool              ret     =
    ((up_serialin(priv,
                     R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_TEND) ==
     R_SCI_B_CSR_TEND);

  return ret;
}

/****************************************************************************
 * Name: up_dma_send_at
 *
 * Description:
 *   Send "length" bytes starting at "buffer", for a port with priv->txdtc
 *   set.  Callers: up_dma_send() (fresh send) and up_txinterrupt()'s
 *   wrap-chain case.
 *
 *   Takes buffer/length as plain arguments, rather than reading
 *   dev->dmatx.buffer/length directly, so the wrap-chain case can send
 *   dev->dmatx.nbuffer/nlength (the second half of a wrapped ring buffer)
 *   without disturbing dev->dmatx.buffer -- uart_xmitchars_done() only
 *   advances dev->xmit.tail while that still points at the first segment
 *   (serial_dma.c).
 *
 *   Section 31.12.2(1) of the RA8M1 User's Manual: setting CCR0.TIE while
 *   TE is already 1 does not itself raise SCIn_TXI, so arming the DTC
 *   alone only works if a TDR->TSR edge is already pending or imminent.
 *   This port always runs non-FIFO when txdtc is set, where CSR.TDRE is a
 *   live level (true exactly when TDR is empty):
 *
 *   TDRE=1 (TDR empty, e.g. first send ever, or a fresh kick with nothing
 *   in flight): arm the DTC for bytes[1:], THEN write byte[0] to TDR
 *   directly -- DTCE must be set before that write, since TDR->TSR can
 *   happen within a few bus cycles and a TXI that latches before DTCE is
 *   set goes to the CPU instead of the DTC.
 *
 *   TDRE=0 (TDR holds a byte not yet at TSR -- always true when called
 *   from up_txinterrupt(), since MRB.DISEL=0 means the CPU interrupt
 *   fires on the DTC's final transfer, the one that just wrote that
 *   pending byte into TDR, not the one reaching TSR -- section 13.2.14):
 *   arm the DTC for the whole new buffer and skip the direct write; the
 *   TDR->TSR edge already pending is what activates it.  Writing byte[0]
 *   directly here instead would overwrite the still-pending byte.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TXDMA
static void up_dma_send_at(struct up_dev_s *priv, uintptr_t buffer,
                            size_t length)
{
  struct dtc_transfer_info_s *info = &priv->dtcinfo;
  uint8_t vector = priv->txirq - RA_IRQ_FIRST;
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(priv->txdtc);
  DEBUGASSERT(length > 0 && length <= 65536);

  up_clean_dcache(buffer, buffer + length);

  flags = enter_critical_section();

  /* On a fresh send (uart_write() -> dmatxavail), callers reach this
   * before uart_enabletxint(), so CCR0.TIE is still 0.  Set it here,
   * before the TDR write below: section 31.12.2(1) gates SCIn_TXI on
   * TIE being 1 at the moment TDRE rises, not retroactively, so setting
   * TIE after the write would miss the edge and leave the DTC armed but
   * never activated.  (Already 1 by the time up_txinterrupt()'s
   * wrap-chain case calls this; the write here is then a no-op.)
   */

  regval  = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
  regval |= R_SCI_B_CCR0_TIE;
  up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);

  /* Normal mode, byte size, SAR increments through the ring buffer, DAR
   * fixed at TDR's low byte.  DISEL=0: one CPU interrupt after the whole
   * count, not one per byte.
   */

  info->mra = R_DTC_MRA_MD_NORMAL | R_DTC_MRA_SZ_BYTE |
              R_DTC_MRA_SM_INCREMENT;
  info->mrb = R_DTC_MRB_DM_FIXED;
  info->dar = priv->scibase + R_SCI_B_TDR_BY_LL_OFFSET;
  info->crb = 0;  /* Unused in normal mode */

  if ((up_serialin(priv, R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_TDRE) != 0)
    {
      /* Idle (TDR empty): write byte[0] directly, and arm the DTC (if
       * more than one byte) for the rest -- see the "idle" case above.
       */

      if (length > 1)
        {
          info->sar = (uint32_t)(buffer + 1);
          info->cra = (uint16_t)(length - 1);

          ra_dtc_configure(vector, info);
          ra_dtc_enable(vector);
        }

      up_serialout(priv, R_SCI_B_TDR_OFFSET,
                   ((uint32_t)(*(FAR uint8_t *)buffer)) &
                   R_SCI_B_TDR_TDAT_MASK);
    }
  else
    {
      /* Not idle: TDR holds a byte that has not yet reached TSR.  Arm
       * the DTC for the whole buffer instead, with no direct write --
       * see the "not idle" case above.
       */

      info->sar = (uint32_t)buffer;
      info->cra = (uint16_t)length;

      ra_dtc_configure(vector, info);
      ra_dtc_enable(vector);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_dma_send
 *
 * Description:
 *   up_dma_send_at() for dev->dmatx.buffer/length -- the .dmasend callback
 *   the generic serial core invokes for a fresh send (dmatxavail's fake
 *   kick, via up_dma_txavail() below).  up_txinterrupt()'s wrap-chain case
 *   calls up_dma_send_at() directly instead, see its own comment.
 *
 ****************************************************************************/

static void up_dma_send(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  up_dma_send_at(priv, (uintptr_t)dev->dmatx.buffer, dev->dmatx.length);
}
#endif

/****************************************************************************
 * Name: up_dma_txavail
 *
 * Description:
 *   Start a new DTC transmit if this port uses one and none is already in
 *   flight.  Called by the serial upper half whenever new data may be
 *   available to send; a no-op for every port except one with
 *   priv->txdtc set.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TXDMA
static void up_dma_txavail(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  /* Without this, a stray TXI could land between the length==0 check and
   * uart_xmitchars_dma() actually arming something, and be missed.
   */

  flags = enter_critical_section();

  if (priv->txdtc && dev->dmatx.length == 0)
    {
      uart_xmitchars_dma(dev);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: up_dma_rxarm
 *
 * Description:
 *   (Re)arm this port's RX DTC vector to move exactly the next byte from
 *   RDR to priv->dtcrxbyte: DTC normal mode, byte size, both source and
 *   destination fixed (a single register, a single scratch variable), one
 *   count.  Rebuilds the whole transfer info every call rather than
 *   assuming it is still 1 from the last time: normal mode's count reads
 *   back as 0 once exhausted, and DTC normal mode treats a stored count of
 *   0 as 65536, not "do nothing" (RA8M1 User's Manual section 17.2.6).
 *
 *   No-ops if DTCE is already 1 (up_rxdrain() calls this unconditionally
 *   on every drain): rewriting a live descriptor while DTCE is still 1
 *   risks tearing an activation already in progress or imminent.
 *
 ****************************************************************************/

#ifdef CONFIG_RA_DTC
static void up_dma_rxarm(struct up_dev_s *priv)
{
  struct dtc_transfer_info_s *info = &priv->dtcrxinfo;
  uint8_t vector = priv->rxirq - RA_IRQ_FIRST;

  if ((getreg32(R_ICU_IELSR(vector)) & R_ICU_IELSR_DTCE) != 0)
    {
      return;
    }

  info->mra = R_DTC_MRA_MD_NORMAL | R_DTC_MRA_SZ_BYTE | R_DTC_MRA_SM_FIXED;
  info->mrb = R_DTC_MRB_DM_FIXED;
  info->sar = priv->scibase + R_SCI_B_RDR_OFFSET;
  info->dar = (uint32_t)(uintptr_t)&priv->dtcrxbyte;
  info->cra = 1;
  info->crb = 0;  /* Unused in normal mode */

  ra_dtc_configure(vector, info);
  ra_dtc_enable(vector);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level SCI initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* Disable all SCIS */

#ifdef TTYS0_DEV
  up_disableallints(TTYS0_DEV.priv, NULL);
#endif
#ifdef TTYS1_DEV
  up_disableallints(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableallints(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  up_disableallints(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
  up_disableallints(TTYS4_DEV.priv, NULL);
#endif
#ifdef TTYS5_DEV
  up_disableallints(TTYS5_DEV.priv, NULL);
#endif

#ifdef HAVE_CONSOLE
  /* Configuration whichever one is the console */

  CONSOLE_DEV.isconsole = true;

  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all SCIs */

#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS4_DEV);
#endif
#ifdef TTYS5_DEV
  uart_register("/dev/ttyS5", &TTYS5_DEV);
#endif
}
