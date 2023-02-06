/****************************************************************************
 * arch/arm/src/sama5/sam_flexcom_serial.c
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
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/sam_flexcom.h"
#include "sam_config.h"
#include "sam_dbgu.h"
#include "sam_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

#undef TTYFC0_DEV
#undef TTYFC1_DEV
#undef TTYFC2_DEV
#undef TTYFC3_DEV
#undef TTYFC4_DEV

#undef USART0_ASSIGNED
#undef USART1_ASSIGNED
#undef USART2_ASSIGNED
#undef USART3_ASSIGNED
#undef USART4_ASSIGNED

#ifdef CONFIG_SAMA5_FLEXCOM_USART

/* Which Flexcom with be ttyFC0/console and which ttyFC1? ttyFC2? ttyFC3?
 * ttyFC4? ttyFC5?
 */

/* First pick the console and ttyFC0.  This could be any of FLEXUS0-4 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_flexus0port /* FLEXUS0 is console */
#    define TTYFC0_DEV          g_flexus0port /* FLEXUS0 is ttyFC0 */
#    define FLEXUS0_ASSIGNED    1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_flexus1port /* FLEXUS1 is console */
#    define TTYFC0_DEV          g_flexus1port /* FLEXUS1 is ttyFC0 */
#    define FLEXUS1_ASSIGNED    1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_flexus2port /* FLEXUS2 is console */
#    define TTYFC0_DEV          g_flexus2port /* FLEXUS2 is ttyFC0 */
#    define FLEXUS2_ASSIGNED    1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_flexus3port /* FLEXUS3 is console */
#    define TTYFC0_DEV          g_flexus3port /* FLEXUS3 is ttyFC0 */
#    define FLEXUS3_ASSIGNED    1
#elif defined(CONFIG_USART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_flexus4port /* FLEXUS4 is console */
#    define TTYFC0_DEV          g_flexus4port /* FLEXUS4 is ttyFC0 */
#    define FLEXUS4_ASSIGNED    1
#else
#  undef CONSOLE_DEV                          /* No console */
#  if defined(CONFIG_USART0_SERIALDRIVER)
#    define TTYFC0_DEV          g_flexus0port /* FLEXUS0 is ttyFC0 */
#    define FLEXUS0_ASSIGNED    1
#  elif defined(CONFIG_USART1_SERIALDRIVER)
#    define TTYFC0_DEV          g_flexus1port /* FLEXUS1 is ttyFC0 */
#    define FLEXUS1_ASSIGNED    1
#  elif defined(CONFIG_USART2_SERIALDRIVER)
#    define TTYFC0_DEV          g_flexus2port /* FLEXUS2 is ttyFC0 */
#    define FLEXUS2_ASSIGNED    1
#  elif defined(CONFIG_USART3_SERIALDRIVER)
#    define TTYFC0_DEV          g_flexus3port /* FLEXUS3 is ttyFC0 */
#    define FLEXUS3_ASSIGNED    1
#  elif defined(CONFIG_USART4_SERIALDRIVER)
#    define TTYFC0_DEV          g_flexus4port /* FLEXUS4 is ttyFC0 */
#    define FLEXUS4_ASSIGNED    1
#  endif
#endif

/* Pick ttyFC1.  This could be any of USART1-4 excluding the console UART. */

#if defined(CONFIG_USART1_SERIALDRIVER) && !defined(FLEXUS1_ASSIGNED)
#  define TTYFC1_DEV           g_flexus1port /* FLEXUS1 is ttyFC1 */
#  define FLEXUS1_ASSIGNED     1
#elif defined(CONFIG_USART2_SERIALDRIVER) && !defined(FLEXUS2_ASSIGNED)
#  define TTYFC1_DEV           g_flexus2port /* FLEXUS2 is ttyFC1 */
#  define FLEXUS2_ASSIGNED     1
#elif defined(CONFIG_USART3_SERIALDRIVER) && !defined(FLEXUS3_ASSIGNED)
#  define TTYFC1_DEV           g_flexus3port /* FLEXUS3 is ttyFC1 */
#  define FLEXUS3_ASSIGNED     1
#elif defined(CONFIG_USART4_SERIALDRIVER) && !defined(FLEXUS4_ASSIGNED)
#  define TTYFC1_DEV           g_flexus4port /* FLEXUS4 is ttyFC1 */
#  define FLEXUS4_ASSIGNED     1
#endif

/* Pick ttyFC2.  This could be one of FLEXUS2-4. It can't be FLEXUS0
 * because that was either assigned as ttyFC0 or ttyFC1.  One of these
 * could also be the console.
 */

#if defined(CONFIG_USART2_SERIALDRIVER) && !defined(FLEXUS2_ASSIGNED)
#  define TTYFC2_DEV           g_flexus2port /* FLEXUS2 is ttyFC2 */
#  define FLEXUS2_ASSIGNED     1
#elif defined(CONFIG_USART3_SERIALDRIVER) && !defined(FLEXUS3_ASSIGNED)
#  define TTYFC2_DEV           g_flexus3port /* FLEXUS3 is ttyFC2 */
#  define FLEXUS3_ASSIGNED     1
#elif defined(CONFIG_USART4_SERIALDRIVER) && !defined(FLEXUS4_ASSIGNED)
#  define TTYFC2_DEV           g_flexus4port /* FLEXUS4 is ttyFC2 */
#  define FLEXUS4_ASSIGNED     1
#endif

/* Pick ttyFC3.  This could be one of FLEXUS3-4. It can't be FLEXUS0-1
 * UART0-1; those have already been assigned to ttyFC0, 1, or 2.  One of
 * FLEXUS2-4 could also be the console.
 */

#if defined(CONFIG_USART3_SERIALDRIVER) && !defined(FLEXUS3_ASSIGNED)
#  define TTYFC3_DEV           g_flexus3port /* FLEXUS3 is ttyFC3 */
#  define FLEXUS3_ASSIGNED     1
#elif defined(CONFIG_USART4_SERIALDRIVER) && !defined(FLEXUS4_ASSIGNED)
#  define TTYFC3_DEV           g_flexus4port /* FLEXUS4 is ttyFC3 */
#  define FLEXUS4_ASSIGNED     1
#endif

/* Pick ttyFC4.  This could be USART4. It can't be one of
 * USART0-2; those have already been assigned to ttyFC0-3.  One of
 * USART3-4 could also be the console.
 */

#if defined(CONFIG_USART4_SERIALDRIVER) && !defined(FLEXUS4_ASSIGNED)
#  define TTYFC4_DEV           g_flexus4port /* USART4 is ttyFC4 */
#  define FLEXUS4_ASSIGNED     1
#endif

/* The Flexcom modules are driven by the peripheral clock (MCK or MCK2). */

#define SAM_USART_CLOCK  BOARD_FLEXCOM_FREQUENCY /* Frequency of the FLEXCOM clock */
#define SAM_MR_USCLKS    FLEXUS_MR_USCLKS_MCK    /* Source = Main clock */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct flexus_dev_s
{
  uint32_t usartbase; /* Base address of USART registers */
  uint32_t baud;      /* Configured baud */
  uint32_t sr;        /* Saved status bits */
  uint8_t  irq;       /* IRQ associated with this USART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  bool     flowc;     /* input flow control (RTS) enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  flexus_interrupt(int irq, void *context, void *arg);
static int  flexus_setup(struct uart_dev_s *dev);
static void flexus_shutdown(struct uart_dev_s *dev);
static int  flexus_attach(struct uart_dev_s *dev);
static void flexus_detach(struct uart_dev_s *dev);
static int  flexus_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  flexus_receive(struct uart_dev_s *dev, unsigned int *status);
static void flexus_rxint(struct uart_dev_s *dev, bool enable);
static bool flexus_rxavailable(struct uart_dev_s *dev);
static void flexus_send(struct uart_dev_s *dev, int ch);
static void flexus_txint(struct uart_dev_s *dev, bool enable);
static bool flexus_txready(struct uart_dev_s *dev);
static bool flexus_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_flexus_ops =
{
  .setup          = flexus_setup,
  .shutdown       = flexus_shutdown,
  .attach         = flexus_attach,
  .detach         = flexus_detach,
  .ioctl          = flexus_ioctl,
  .receive        = flexus_receive,
  .rxint          = flexus_rxint,
  .rxavailable    = flexus_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = flexus_send,
  .txint          = flexus_txint,
  .txready        = flexus_txready,
  .txempty        = flexus_txempty,
};

/* I/O buffers */

#ifdef CONFIG_USART0_SERIALDRIVER
static char g_flexus0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_flexus0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef CONFIG_USART1_SERIALDRIVER
static char g_flexus1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_flexus1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef CONFIG_SAMA5_FLEXCOM2_USART
static char g_flexus2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_flexus2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef CONFIG_USART3_SERIALDRIVER
static char g_flexus3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_flexus3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif
#ifdef CONFIG_USART4_SERIALDRIVER
static char g_flexus4rxbuffer[CONFIG_USART4_RXBUFSIZE];
static char g_flexus4txbuffer[CONFIG_USART4_TXBUFSIZE];
#endif

/* This describes the state of the USART0 port. */

#ifdef CONFIG_USART0_SERIALDRIVER
static struct flexus_dev_s g_flexus0priv =
{
  .usartbase      = SAM_FLEXCOM0_VBASE,
  .baud           = CONFIG_USART0_BAUD,
  .irq            = SAM_IRQ_FLEXCOM0,
  .parity         = CONFIG_USART0_PARITY,
  .bits           = CONFIG_USART0_BITS,
  .stopbits2      = CONFIG_USART0_2STOP,
#if defined(CONFIG_USART0_OFLOWCONTROL) || defined(CONFIG_USART0_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_flexus0port =
{
  .recv     =
  {
    .size   = CONFIG_USART0_RXBUFSIZE,
    .buffer = g_flexus0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART0_TXBUFSIZE,
    .buffer = g_flexus0txbuffer,
  },
  .ops      = &g_flexus_ops,
  .priv     = &g_flexus0priv,
};
#endif

/* This describes the state of the USART1 port. */

#ifdef CONFIG_USART1_SERIALDRIVER
static struct flexus_dev_s g_flexus1priv =
{
  .usartbase      = SAM_FLEXCOM1_VBASE,
  .baud           = CONFIG_USART1_BAUD,
  .irq            = SAM_IRQ_FLEXCOM1,
  .parity         = CONFIG_USART1_PARITY,
  .bits           = CONFIG_USART1_BITS,
  .stopbits2      = CONFIG_USART1_2STOP,
#if defined(CONFIG_USART1_OFLOWCONTROL) || defined(CONFIG_USART1_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_flexus1port =
{
  .recv     =
  {
    .size   = CONFIG_USART1_RXBUFSIZE,
    .buffer = g_flexus1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART1_TXBUFSIZE,
    .buffer = g_flexus1txbuffer,
  },
  .ops      = &g_flexus_ops,
  .priv     = &g_flexus1priv,
};
#endif

/* This describes the state of the USART2 port. */

#ifdef CONFIG_SAMA5_FLEXCOM2_USART
static struct flexus_dev_s g_flexus2priv =
{
  .usartbase      = SAM_FLEXCOM2_VBASE,
  .baud           = CONFIG_USART2_BAUD,
  .irq            = SAM_IRQ_FLEXCOM2,
  .parity         = CONFIG_USART2_PARITY,
  .bits           = CONFIG_USART2_BITS,
  .stopbits2      = CONFIG_USART2_2STOP,
#if defined(CONFIG_USART2_OFLOWCONTROL) || defined(CONFIG_USART2_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_flexus2port =
{
  .recv     =
  {
    .size   = CONFIG_USART2_RXBUFSIZE,
    .buffer = g_flexus2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART2_TXBUFSIZE,
    .buffer = g_flexus2txbuffer,
  },
  .ops      = &g_flexus_ops,
  .priv     = &g_flexus2priv,
};
#endif

/* This describes the state of the USART3 port. */

#ifdef CONFIG_USART3_SERIALDRIVER
static struct flexus_dev_s g_flexus3priv =
{
  .usartbase      = SAM_FLEXCOM3_VBASE,
  .baud           = CONFIG_USART3_BAUD,
  .irq            = SAM_IRQ_FLEXCOM3,
  .parity         = CONFIG_USART3_PARITY,
  .bits           = CONFIG_USART3_BITS,
  .stopbits2      = CONFIG_USART3_2STOP,
#if defined(CONFIG_USART3_OFLOWCONTROL) || defined(CONFIG_USART3_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_flexus3port =
{
  .recv     =
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_flexus3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_flexus3txbuffer,
  },
  .ops      = &g_flexus_ops,
  .priv     = &g_flexus3priv,
};
#endif

/* This describes the state of the USART4 port. */

#ifdef CONFIG_USART4_SERIALDRIVER
static struct flexus_dev_s g_flexus4priv =
{
  .usartbase      = SAM_FLEXCOM4_VBASE,
  .baud           = CONFIG_USART4_BAUD,
  .irq            = SAM_IRQ_FLEXCOM4,
  .parity         = CONFIG_USART4_PARITY,
  .bits           = CONFIG_USART4_BITS,
  .stopbits2      = CONFIG_USART4_2STOP,
#if defined(CONFIG_USART4_OFLOWCONTROL) || defined(CONFIG_USART4_IFLOWCONTROL)
  .flowc          = true,
#endif
};

static uart_dev_t g_flexus4port =
{
  .recv     =
  {
    .size   = CONFIG_USART4_RXBUFSIZE,
    .buffer = g_flexus4rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART4_TXBUFSIZE,
    .buffer = g_flexus4txbuffer,
  },
  .ops      = &g_flexus_ops,
  .priv     = &g_flexus4priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flexus_serialin
 ****************************************************************************/

static inline uint32_t flexus_serialin(struct flexus_dev_s *priv, int offset)
{
  return getreg32(priv->usartbase + offset);
}

/****************************************************************************
 * Name: flexus_serialout
 ****************************************************************************/

static inline void flexus_serialout(struct flexus_dev_s *priv, int offset,
                                    uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: flexus_restoreusartint
 ****************************************************************************/

static inline void flexus_restoreusartint(struct flexus_dev_s *priv,
                                          uint32_t imr)
{
  /* Restore the previous interrupt state (assuming all interrupts
   * disabled)
   */

  flexus_serialout(priv, SAM_FLEXUS_IER_OFFSET, imr);
}

/****************************************************************************
 * Name: flexus_disableallints
 ****************************************************************************/

static void flexus_disableallints(struct flexus_dev_s *priv, uint32_t *imr)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = enter_critical_section();

  /* Return the current interrupt state */

  if (imr)
    {
      *imr = flexus_serialin(priv, SAM_FLEXUS_IMR_OFFSET);
    }

  /* Disable all interrupts */

  flexus_serialout(priv, SAM_FLEXUS_IDR_OFFSET, FLEXUS_INT_ALLINTS);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: flexus_interrupt
 *
 * Description:
 *   This is the common USART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int flexus_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct flexus_dev_s *priv;
  uint32_t pending;
  uint32_t imr;
  int passes;
  bool handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct flexus_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or, until we have
   * been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the UART/USART status (we are only interested in the unmasked
       * interrupts).
       */

      priv->sr = flexus_serialin(priv, SAM_FLEXUS_CSR_OFFSET); /* Save for error reporting */
      imr      = flexus_serialin(priv, SAM_FLEXUS_IMR_OFFSET); /* Interrupt mask */
      pending  = priv->sr & imr;                               /* Mask out disabled interrupt sources */

      /* Handle an incoming, receive byte.  RXRDY: At least one complete
       * character has been received and US_RHR has not yet been read.
       */

      if ((pending & FLEXUS_INT_RXRDY) != 0)
        {
          /* Received data ready... process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

      /* Handle outgoing, transmit bytes. XRDY: There is no character in the
       * US_THR.
       */

      if ((pending & FLEXUS_INT_TXRDY) != 0)
        {
          /* Transmit data register empty ... process outgoing bytes */

          uart_xmitchars(dev);
          handled = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: flexus_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int flexus_setup(struct uart_dev_s *dev)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t regval;

  /* Note: The logic here depends on the fact that that the USART module
   * was enabled and the pins were configured in sam_lowsetup().
   */

  /* The shutdown method will put the UART in a known, disabled state */

  flexus_shutdown(dev);

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  /* Setting the USART to operate with hardware handshaking is performed by
   * writing the USART_MODE field in the Mode Register (US_MR) to the value
   * 0x2. ... Using this mode requires using the PDC or DMAC channel for
   * reception. The transmitter can handle hardware handshaking in any case.
   */

  if (priv->flowc)
    {
      /* Enable hardware flow control and MCK as the timing source */

      regval = (FLEXUS_MR_MODE_HWHS | SAM_MR_USCLKS |
                FLEXUS_MR_CHMODE_NORMAL);
    }
  else
#endif
    {
      /* Set up the mode register.  Start with normal UART mode and the MCK
       * as the timing source
       */

      regval = (FLEXUS_MR_MODE_NORMAL | SAM_MR_USCLKS |
                FLEXUS_MR_CHMODE_NORMAL);
    }

  /* OR in settings for the selected number of bits */

  if (priv->bits == 5)
    {
      regval |= FLEXUS_MR_CHRL_5BITS; /* 5 bits */
    }
  else if (priv->bits == 6)
    {
      regval |= FLEXUS_MR_CHRL_6BITS;  /* 6 bits */
    }
  else if (priv->bits == 7)
    {
      regval |= FLEXUS_MR_CHRL_7BITS; /* 7 bits */
    }
  else if (priv->bits == 9)
    {
      regval |= FLEXUS_MR_MODE9; /* 9 bits */
    }
  else /* if (priv->bits == 8) */
    {
      regval |= FLEXUS_MR_CHRL_8BITS; /* 8 bits (default) */
    }

  /* OR in settings for the selected parity */

  if (priv->parity == 1)
    {
      regval |= FLEXUS_MR_PAR_ODD;
    }
  else if (priv->parity == 2)
    {
      regval |= FLEXUS_MR_PAR_EVEN;
    }
  else
    {
      regval |= FLEXUS_MR_PAR_NONE;
    }

  /* OR in settings for the number of stop bits */

  if (priv->stopbits2)
    {
      regval |= FLEXUS_MR_NBSTOP_2;
    }
  else
    {
      regval |= FLEXUS_MR_NBSTOP_1;
    }

  /* And save the new mode register value */

  flexus_serialout(priv, SAM_FLEXUS_MR_OFFSET, regval);

  /* Configure the console baud.  NOTE: Oversampling by 8 is not supported.
   * This may limit BAUD rates for lower USART clocks.
   */

  regval  = (SAM_USART_CLOCK + (priv->baud << 3)) / (priv->baud << 4);
  flexus_serialout(priv, SAM_FLEXUS_BRGR_OFFSET, regval);

  /* Enable receiver & transmitter */

  flexus_serialout(priv, SAM_FLEXUS_CR_OFFSET,
                   FLEXUS_CR_RXEN | FLEXUS_CR_TXEN);
#endif
  return OK;
}

/****************************************************************************
 * Name: flexus_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void flexus_shutdown(struct uart_dev_s *dev)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;

  /* Reset and disable receiver and transmitter */

  flexus_serialout(priv, SAM_FLEXUS_CR_OFFSET,
                   (FLEXUS_CR_RSTRX | FLEXUS_CR_RSTTX | FLEXUS_CR_RXDIS |
                    FLEXUS_CR_TXDIS));

  /* Disable all interrupts */

  flexus_disableallints(priv, NULL);
}

/****************************************************************************
 * Name: flexus_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method
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

static int flexus_attach(struct uart_dev_s *dev)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, flexus_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the USART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: flexus_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void flexus_detach(struct uart_dev_s *dev)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: flexus_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int flexus_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct flexus_dev_s *user = (struct flexus_dev_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct flexus_dev_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return parity */

        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                            ((priv->parity == 1) ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= (priv->stopbits2) ? CSTOPB : 0;

        /* Return flow control */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        termiosp->c_cflag |= (priv->flowc) ? (CCTS_OFLOW | CRTS_IFLOW): 0;
#endif
        /* Return baud */

        cfsetispeed(termiosp, priv->baud);

        /* Return number of bits */

        switch (priv->bits)
          {
          case 5:
            termiosp->c_cflag |= CS5;
            break;

          case 6:
            termiosp->c_cflag |= CS6;
            break;

          case 7:
            termiosp->c_cflag |= CS7;
            break;

          default:
          case 8:
            termiosp->c_cflag |= CS8;
            break;

          case 9:
            termiosp->c_cflag |= CS8; /* CS9 */
            break;
          }
      }
      break;

    case TCSETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;
        uint32_t baud;
        uint32_t imr;
        uint8_t parity;
        uint8_t nbits;
        bool stop2;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        bool flowc;
#endif

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Decode baud. */

        ret = OK;
        baud = cfgetispeed(termiosp);

        /* Decode number of bits */

        switch (termiosp->c_cflag & CSIZE)
          {
          case CS5:
            nbits = 5;
            break;

          case CS6:
            nbits = 6;
            break;

          case CS7:
            nbits = 7;
            break;

          case CS8:
            nbits = 8;
            break;
#if 0
          case CS9:
            nbits = 9;
            break;
#endif
          default:
            ret = -EINVAL;
            break;
          }

        /* Decode parity */

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        /* Decode stop bits */

        stop2 = (termiosp->c_cflag & CSTOPB) != 0;

        /* Decode flow control */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        flowc = (termiosp->c_cflag & (CCTS_OFLOW | CRTS_IFLOW)) != 0;
#endif
        /* Verify that all settings are valid before committing */

        if (ret == OK)
          {
            /* Commit */

            priv->baud      = baud;
            priv->parity    = parity;
            priv->bits      = nbits;
            priv->stopbits2 = stop2;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
            priv->flowc     = flowc;
#endif
            /* Effect the changes immediately - note that we do not
             * implement TCSADRAIN / TCSAFLUSH
             */

            flexus_disableallints(priv, &imr);
            ret = flexus_setup(dev);

            /* Restore the interrupt state */

            flexus_restoreusartint(priv, imr);
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: flexus_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int flexus_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;

  /* Return the error information in the saved status */

  *status  = priv->sr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return (int)(flexus_serialin(priv, SAM_FLEXUS_RHR_OFFSET) & 0xff);
}

/****************************************************************************
 * Name: flexus_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void flexus_rxint(struct uart_dev_s *dev, bool enable)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      flexus_serialout(priv, SAM_FLEXUS_IER_OFFSET, FLEXUS_INT_RXRDY);
#endif
    }
  else
    {
      flexus_serialout(priv, SAM_FLEXUS_IDR_OFFSET, FLEXUS_INT_RXRDY);
    }
}

/****************************************************************************
 * Name: flexus_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool flexus_rxavailable(struct uart_dev_s *dev)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;
  uint32_t value = flexus_serialin(priv, SAM_FLEXUS_CSR_OFFSET);
  return (value & FLEXUS_INT_RXRDY) != 0;
}

/****************************************************************************
 * Name: flexus_send
 *
 * Description:
 *   This method will send one byte on the UART/USART
 *
 ****************************************************************************/

static void flexus_send(struct uart_dev_s *dev, int ch)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;
  flexus_serialout(priv, SAM_FLEXUS_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: flexus_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void flexus_txint(struct uart_dev_s *dev, bool enable)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      flexus_serialout(priv, SAM_FLEXUS_IER_OFFSET, FLEXUS_INT_TXRDY);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);

#endif
    }
  else
    {
      /* Disable the TX interrupt */

      flexus_serialout(priv, SAM_FLEXUS_IDR_OFFSET, FLEXUS_INT_TXRDY);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: flexus_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool flexus_txready(struct uart_dev_s *dev)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;
  uint32_t value = flexus_serialin(priv, SAM_FLEXUS_CSR_OFFSET);
  return (value & FLEXUS_INT_TXRDY) != 0;
}

/****************************************************************************
 * Name: flexus_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool flexus_txempty(struct uart_dev_s *dev)
{
  struct flexus_dev_s *priv = (struct flexus_dev_s *)dev->priv;
  uint32_t value = flexus_serialin(priv, SAM_FLEXUS_CSR_OFFSET);
  return (value & FLEXUS_INT_TXEMPTY) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: flexus_earlyserialinit
 *
 * Description:
 *   Performs the low level Flexcom USART initialization early so that the
 *   Flexcom serial console will be available during bootup.  This must be
 *   called before flexus_serialinit.
 *
 ****************************************************************************/

void flexus_earlyserialinit(void)
{
  /* NOTE:  All PIO configuration for the USARTs was performed in
   * sam_lowsetup
   */

  /* Disable the USART */

#ifdef TTYFC0_DEV
  /* Select USART mode for the Flexcom */

  flexus_serialout(TTYFC0_DEV.priv,
                   SAM_FLEX_MR_OFFSET, FLEX_MR_OPMODE_USART);

  /* Disable the USART */

  flexus_disableallints(TTYFC0_DEV.priv, NULL);
#endif
#ifdef TTYFC1_DEV
  /* Select USART mode for the Flexcom */

  flexus_serialout(TTYFC1_DEV.priv,
                   SAM_FLEX_MR_OFFSET, FLEX_MR_OPMODE_USART);

  /* Disable the USART */

  flexus_disableallints(TTYFC1_DEV.priv, NULL);
#endif
#ifdef TTYFC2_DEV
  /* Select USART mode for the Flexcom */

  flexus_serialout(TTYFC2_DEV.priv,
                   SAM_FLEX_MR_OFFSET, FLEX_MR_OPMODE_USART);

  /* Disable the USART */

  flexus_disableallints(TTYFC2_DEV.priv, NULL);
#endif
#ifdef TTYFC3_DEV
  /* Select USART mode for the Flexcom */

  flexus_serialout(TTYFC3_DEV.priv,
                   SAM_FLEX_MR_OFFSET, FLEX_MR_OPMODE_USART);

  /* Disable the USART */

  flexus_disableallints(TTYFC3_DEV.priv, NULL);
#endif
#ifdef TTYFC4_DEV
  /* Select USART mode for the Flexcom */

  flexus_serialout(TTYFC4_DEV.priv,
                   SAM_FLEX_MR_OFFSET, FLEX_MR_OPMODE_USART);

  /* Disable the USART */

  flexus_disableallints(TTYFC4_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef SAMA5_HAVE_FLEXCOM_CONSOLE
  CONSOLE_DEV.isconsole = true;
  flexus_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: flexus_serialinit
 *
 * Description:
 *   Register Flexcom serial console and serial ports.  This assumes that
 *   flexus_earlyserialinit was called previously.
 *
 ****************************************************************************/

void flexus_serialinit(void)
{
  /* Register the console */

#ifdef SAMA5_HAVE_FLEXCOM_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs/USARTs */

#ifdef TTYFC0_DEV
  uart_register("/dev/ttyFC0", &TTYFC0_DEV);
#endif
#ifdef TTYFC1_DEV
  uart_register("/dev/ttyFC1", &TTYFC1_DEV);
#endif
#ifdef TTYFC2_DEV
  uart_register("/dev/ttyFC2", &TTYFC2_DEV);
#endif
#ifdef TTYFC3_DEV
  uart_register("/dev/ttyFC3", &TTYFC3_DEV);
#endif
#ifdef TTYFC4_DEV
  uart_register("/dev/ttyFC4", &TTYFC4_DEV);
#endif
}

#endif /* USE_SERIALDRIVER */
#endif /* SAMA5_HAVE_FLEXCOM_USART */
