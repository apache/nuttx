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
};

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
 * Name: up_rxinterrupt
 *
 * Description:
 *   This is the common SCI RX interrupt handler.
 *
 ****************************************************************************/

static int up_rxinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;

  ra_clear_ir(irq);
  uart_recvchars(dev);
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

  ra_clear_ir(irq);
  uart_xmitchars(dev);

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

  /* Save for error reporting */

  priv->sr = up_serialin(priv, R_SCI_B_CSR_OFFSET) & SCI_UART_ERR_BITS;

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
  bool              ret     =
    ((up_serialin(priv,
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
