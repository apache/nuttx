/****************************************************************************
 * arch/arm/src/samd2l2/sam_serial.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "sam_config.h"
#include "sam_usart.h"
#include "sam_lowputc.h"
#include "sam_serial.h"

#ifdef SAMD2L2_HAVE_USART

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which USART with be tty0/console and which tty1? tty2? tty3? tty4? tty5? */

/* First pick the console and ttys0.  This could be any of USART0-5 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart0port /* USART0 is console */
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED     1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart1port /* USART1 is console */
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED      1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart2port /* USART2 is console */
#    define TTYS0_DEV           g_usart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart3port /* USART3 is console */
#    define TTYS0_DEV           g_usart3port /* USART3 is ttyS0 */
#    define USART3_ASSIGNED     1
#elif defined(CONFIG_USART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart4port /* USART4 is console */
#    define TTYS0_DEV           g_usart4port /* USART4 is ttyS0 */
#    define USART4_ASSIGNED     1
#elif defined(CONFIG_USART5_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_usart5port /* USART5 is console */
#    define TTYS0_DEV           g_usart5port /* USART5 is ttyS0 */
#    define USART5_ASSIGNED     1
#else
#  undef CONSOLE_DEV                         /* No console */
#  if defined(SAMD2L2_HAVE_USART0)
#    define TTYS0_DEV           g_usart0port /* USART0 is ttyS0 */
#    define USART0_ASSIGNED      1
#  elif defined(SAMD2L2_HAVE_USART1)
#    define TTYS0_DEV           g_usart1port /* USART1 is ttyS0 */
#    define USART1_ASSIGNED      1
#  elif defined(SAMD2L2_HAVE_USART2)
#    define TTYS0_DEV           g_usart2port /* USART2 is ttyS0 */
#    define USART2_ASSIGNED     1
#  elif defined(SAMD2L2_HAVE_USART3)
#    define TTYS0_DEV           g_usart3port /* USART3 is ttyS0 */
#    define USART3_ASSIGNED     1
#  elif defined(SAMD2L2_HAVE_USART4)
#    define TTYS0_DEV           g_usart4port /* USART4 is ttyS0 */
#    define USART4_ASSIGNED     1
#  elif defined(SAMD2L2_HAVE_USART5)
#    define TTYS0_DEV           g_usart5port /* USART5 is ttyS0 */
#    define USART5_ASSIGNED     1
#  endif
#endif

/* Pick ttys1.  This could be any of USART0-5 excluding the console USART. */

#if defined(SAMD2L2_HAVE_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS1_DEV           g_usart0port /* USART0 is ttyS1 */
#  define USART0_ASSIGNED      1
#elif defined(SAMD2L2_HAVE_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS1_DEV           g_usart1port /* USART1 is ttyS1 */
#  define USART1_ASSIGNED      1
#elif defined(SAMD2L2_HAVE_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS1_DEV           g_usart2port /* USART2 is ttyS1 */
#  define USART2_ASSIGNED     1
#elif defined(SAMD2L2_HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS1_DEV           g_usart3port /* USART3 is ttyS1 */
#  define USART3_ASSIGNED     1
#elif defined(SAMD2L2_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS1_DEV           g_usart4port /* USART4 is ttyS1 */
#  define USART4_ASSIGNED     1
#elif defined(SAMD2L2_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS1_DEV           g_usart5port /* USART5 is ttyS1 */
#  define USART5_ASSIGNED     1
#endif

/* Pick ttys2.  This could be one of USART1-5. It can't be USART0
 * because that was either assigned as ttyS0 or ttys1.  One of these
 * could also be the console.
 */

#if defined(SAMD2L2_HAVE_USART1) && !defined(USART1_ASSIGNED)
#  define TTYS2_DEV           g_usart1port /* USART1 is ttyS2 */
#  define USART1_ASSIGNED      1
#elif defined(SAMD2L2_HAVE_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS2_DEV           g_usart2port /* USART2 is ttyS2 */
#  define USART2_ASSIGNED     1
#elif defined(SAMD2L2_HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS2_DEV           g_usart3port /* USART3 is ttyS2 */
#  define USART3_ASSIGNED     1
#elif defined(SAMD2L2_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS2_DEV           g_usart4port /* USART4 is ttyS2 */
#  define USART4_ASSIGNED     1
#elif defined(SAMD2L2_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS2_DEV           g_usart5port /* USART5 is ttyS2 */
#  define USART5_ASSIGNED     1
#endif

/* Pick ttys3. This could be one of USART2-5. It can't be USART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * USART2-5 could also be the console.
 */

#if defined(SAMD2L2_HAVE_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS3_DEV           g_usart2port /* USART2 is ttyS3 */
#  define USART2_ASSIGNED     1
#elif defined(SAMD2L2_HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS3_DEV           g_usart3port /* USART3 is ttyS3 */
#  define USART3_ASSIGNED     1
#elif defined(SAMD2L2_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS3_DEV           g_usart4port /* USART4 is ttyS3 */
#  define USART4_ASSIGNED     1
#elif defined(SAMD2L2_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS3_DEV           g_usart5port /* USART5 is ttyS3 */
#  define USART5_ASSIGNED     1
#endif

/* Pick ttys4. This could be one of USART3-5. It can't be USART0-2
 * because those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * USART3-5 could also be the console.
 */

#if defined(SAMD2L2_HAVE_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS4_DEV           g_usart3port /* USART3 is ttyS4 */
#  define USART3_ASSIGNED      1
#elif defined(SAMD2L2_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS4_DEV           g_usart4port /* USART4 is ttyS4 */
#  define USART4_ASSIGNED      1
#elif defined(SAMD2L2_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS4_DEV           g_usart5port /* USART5 is ttyS4 */
#  define USART5_ASSIGNED      1
#endif

/* Pick ttys5. This could be one of USART4-5. It can't be USART0-3
 * because those have already been assigned to ttsyS0, 1, 2, 3 or 4.
 * One of USART4-5 could also be the console.
 */

#if defined(SAMD2L2_HAVE_USART4) && !defined(USART4_ASSIGNED)
#  define TTYS5_DEV           g_usart4port /* USART4 is ttyS5 */
#  define USART4_ASSIGNED      1
#elif defined(SAMD2L2_HAVE_USART5) && !defined(USART5_ASSIGNED)
#  define TTYS5_DEV           g_usart5port /* USART5 is ttyS5 */
#  define USART5_ASSIGNED      1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_dev_s
{
  /* Common USART configuration */

  const struct sam_usart_config_s * const config;

  /* Information unique to the serial driver */

#ifdef HAVE_RS485
  const uint32_t rs485_dir_gpio;     /* USART RS-485 DIR GPIO pin configuration */
  const bool     rs485_dir_polarity; /* USART RS-485 DIR pin state for TX enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Support functions */

static inline uint8_t
            sam_serialin8(struct sam_dev_s *priv, int offset);
static inline void
            sam_serialout8(struct sam_dev_s *priv, int offset,
              uint8_t regval);
static inline uint16_t
            sam_serialin16(struct sam_dev_s *priv, int offset);
static inline void
            sam_serialout16(struct sam_dev_s *priv, int offset,
              uint16_t regval);
static void sam_disableallints(struct sam_dev_s *priv);
static int  sam_interrupt(int irq, void *context, void *arg);

/* UART methods */

static int  sam_setup(struct uart_dev_s *dev);
static void sam_shutdown(struct uart_dev_s *dev);
static int  sam_attach(struct uart_dev_s *dev);
static void sam_detach(struct uart_dev_s *dev);
static int  sam_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  sam_receive(struct uart_dev_s *dev, unsigned int *status);
static void sam_rxint(struct uart_dev_s *dev, bool enable);
static bool sam_rxavailable(struct uart_dev_s *dev);
static void sam_send(struct uart_dev_s *dev, int ch);
static void sam_txint(struct uart_dev_s *dev, bool enable);
static bool sam_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = sam_setup,
  .shutdown       = sam_shutdown,
  .attach         = sam_attach,
  .detach         = sam_detach,
  .ioctl          = sam_ioctl,
  .receive        = sam_receive,
  .rxint          = sam_rxint,
  .rxavailable    = sam_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = sam_send,
  .txint          = sam_txint,
  .txready        = sam_txempty,
  .txempty        = sam_txempty,
};

/* I/O buffers */

#ifdef SAMD2L2_HAVE_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef SAMD2L2_HAVE_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef SAMD2L2_HAVE_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef SAMD2L2_HAVE_USART3
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif
#ifdef SAMD2L2_HAVE_USART4
static char g_usart4rxbuffer[CONFIG_USART4_RXBUFSIZE];
static char g_usart4txbuffer[CONFIG_USART4_TXBUFSIZE];
#endif
#ifdef SAMD2L2_HAVE_USART5
static char g_usart5rxbuffer[CONFIG_USART5_RXBUFSIZE];
static char g_usart5txbuffer[CONFIG_USART5_TXBUFSIZE];
#endif

/* This describes the state of the USART0 port. */

#ifdef SAMD2L2_HAVE_USART0
static struct sam_dev_s g_usart0priv =
{
  .config             = &g_usart0config,
#ifdef CONFIG_USART0_RS485MODE
  .rs485_dir_gpio     = GPIO_USART0_RS485_DIR,
#  if (CONFIG_USART0_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static uart_dev_t g_usart0port =
{
  .recv     =
  {
    .size   = CONFIG_USART0_RXBUFSIZE,
    .buffer = g_usart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART0_TXBUFSIZE,
    .buffer = g_usart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart0priv,
};
#endif

/* This describes the state of the USART1 port. */

#ifdef SAMD2L2_HAVE_USART1
static struct sam_dev_s g_usart1priv =
{
  .config             = &g_usart1config,
#ifdef CONFIG_USART1_RS485MODE
  .rs485_dir_gpio     = GPIO_USART1_RS485_DIR,
#  if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static uart_dev_t g_usart1port =
{
  .recv     =
  {
    .size   = CONFIG_USART1_RXBUFSIZE,
    .buffer = g_usart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART1_TXBUFSIZE,
    .buffer = g_usart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart1priv,
};
#endif

/* This describes the state of the USART2 port. */

#ifdef SAMD2L2_HAVE_USART2
static struct sam_dev_s g_usart2priv =
{
  .config             = &g_usart2config,
#ifdef CONFIG_USART2_RS485MODE
  .rs485_dir_gpio     = GPIO_USART2_RS485_DIR,
#  if (CONFIG_USART2_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static uart_dev_t g_usart2port =
{
  .recv     =
  {
    .size   = CONFIG_USART2_RXBUFSIZE,
    .buffer = g_usart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART2_TXBUFSIZE,
    .buffer = g_usart2txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart2priv,
};
#endif

/* This describes the state of the USART3 port. */

#ifdef SAMD2L2_HAVE_USART3
static struct sam_dev_s g_usart3priv =
{
  .config             = &g_usart3config,
#ifdef CONFIG_USART3_RS485MODE
  .rs485_dir_gpio     = GPIO_USART3_RS485_DIR,
#  if (CONFIG_USART3_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static uart_dev_t g_usart3port =
{
  .recv     =
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_usart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_usart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart3priv,
};
#endif

/* This describes the state of the USART4 port. */

#ifdef SAMD2L2_HAVE_USART4
static struct sam_dev_s g_usart4priv =
{
  .config             = &g_usart4config,
#ifdef CONFIG_USART4_RS485MODE
  .rs485_dir_gpio     = GPIO_USART4_RS485_DIR,
#  if (CONFIG_USART4_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static uart_dev_t g_usart4port =
{
  .recv     =
  {
    .size   = CONFIG_USART4_RXBUFSIZE,
    .buffer = g_usart4rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART4_TXBUFSIZE,
    .buffer = g_usart4txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart4priv,
};
#endif

/* This describes the state of the USART5 port. */

#ifdef SAMD2L2_HAVE_USART5
static struct sam_dev_s g_usart5priv =
{
  .config             = &g_usart5config,
#ifdef CONFIG_USART5_RS485MODE
  .rs485_dir_gpio     = GPIO_USART5_RS485_DIR,
#  if (CONFIG_USART5_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static uart_dev_t g_usart5port =
{
  .recv     =
  {
    .size   = CONFIG_USART5_RXBUFSIZE,
    .buffer = g_usart5rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART5_TXBUFSIZE,
    .buffer = g_usart5txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart5priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_serialin8
 ****************************************************************************/

static inline uint8_t sam_serialin8(struct sam_dev_s *priv, int offset)
{
  return getreg8(priv->config->base + offset);
}

/****************************************************************************
 * Name: sam_serialout8
 ****************************************************************************/

static inline void sam_serialout8(struct sam_dev_s *priv, int offset,
                                  uint8_t regval)
{
  putreg8(regval, priv->config->base + offset);
}

/****************************************************************************
 * Name: sam_serialin16
 ****************************************************************************/

static inline uint16_t sam_serialin16(struct sam_dev_s *priv, int offset)
{
  return getreg16(priv->config->base + offset);
}

/****************************************************************************
 * Name: sam_serialout16
 ****************************************************************************/

static inline void sam_serialout16(struct sam_dev_s *priv, int offset,
                                   uint16_t regval)
{
  putreg16(regval, priv->config->base + offset);
}

/****************************************************************************
 * Name: sam_disableallints
 ****************************************************************************/

static void sam_disableallints(struct sam_dev_s *priv)
{
  /* Disable all interrupts */

  sam_serialout8(priv, SAM_USART_INTENCLR_OFFSET, USART_INT_ALL);
}

/****************************************************************************
 * Name: sam_interrupt
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int sam_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct sam_dev_s *priv;
  uint8_t pending;
  uint8_t intflag;
  uint8_t inten;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct sam_dev_s *)dev->priv;

  /* Get the set of pending USART interrupts (we are only interested in the
   * unmasked interrupts).
   */

  intflag = sam_serialin8(priv, SAM_USART_INTFLAG_OFFSET);
  inten   = sam_serialin8(priv, SAM_USART_INTENCLR_OFFSET);
  pending = intflag & inten;

#ifdef HAVE_RS485
  /* Transmission of whole buffer is over - TXC is set.
   * Note - this should be first, to have the most recent TC bit value from
   * SR register - sending data affects TXC, but without refresh we will not
   * know that...
   */

  if ((pending & USART_INT_TXC) != 0)
    {
      sam_portwrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
      sam_serialout8(priv, SAM_USART_INTENCLR_OFFSET, USART_INT_TXC);
    }
#endif

  /* Handle an incoming, receive byte.  The RXC flag is set when there is
   * unread data in DATA register.  This flag is cleared by reading the DATA
   * register (or by disabling the receiver).
   */

  if ((pending & USART_INT_RXC) != 0)
    {
      /* Received data ready... process incoming bytes */

       uart_recvchars(dev);
    }

  /* Handle outgoing, transmit bytes. The DRE flag is set when the DATA
   * register is empty and ready to be written.  This flag is cleared by
   * writing new data to the DATA register.  If there is no further data to
   * be transmitted,  the serial driver will disable TX interrupts, prohibit
   * further interrupts until TX interrupts are re-enabled.
   */

  if ((pending & USART_INT_DRE) != 0)
    {
      /* Transmit data register empty ... process outgoing bytes */

      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int sam_setup(struct uart_dev_s *dev)
{
  int ret = 0;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  /* Configure the SERCOM as a USART.  Don't reconfigure the console UART;
   * that was already done in sam_lowputc.c.
   */

  if (!dev->isconsole)
    {
      ret = sam_usart_initialize(priv->config);
      if (ret >= 0)
        {
          sam_usart_enable(priv->config);
        }
    }
#endif

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      sam_configport(priv->rs485_dir_gpio);
      sam_portwrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: sam_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial port is
 *   closed.  The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void sam_shutdown(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  /* Resetting the SERCOM restores all registers to the reget state and
   * disables the SERCOM.  Ignore any requests to shutown the console
   * device (shouldn't happen).
   */

  if (!dev->isconsole)
    {
      sam_usart_reset(priv->config);
    }
}

/****************************************************************************
 * Name: sam_attach
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

static int sam_attach(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  const struct sam_usart_config_s * const config = priv->config;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(config->irq, sam_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the USART
       */

      up_enable_irq(config->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void sam_detach(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  const struct sam_usart_config_s * const config = priv->config;

  /* Disable interrupts at the SERCOM device and at the NVIC */

  sam_disableallints(priv);
  up_disable_irq(config->irq);

  /* Detach the interrupt handler */

  irq_detach(config->irq);
}

/****************************************************************************
 * Name: sam_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int sam_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct sam_dev_s *user = (struct sam_dev_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct sam_dev_s));
           }
       }
       break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: sam_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int sam_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  /* Return read status */

  *status = (uint32_t)sam_serialin16(priv, SAM_USART_STATUS_OFFSET);

  /* Then return the actual received byte */

  return (int)sam_serialin16(priv, SAM_USART_DATA_OFFSET);
}

/****************************************************************************
 * Name: sam_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void sam_rxint(struct uart_dev_s *dev, bool enable)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data
       * register
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      sam_serialout8(priv, SAM_USART_INTENSET_OFFSET, USART_INT_RXC);
#endif
    }
  else
    {
      sam_serialout8(priv, SAM_USART_INTENCLR_OFFSET, USART_INT_RXC);
    }
}

/****************************************************************************
 * Name: sam_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool sam_rxavailable(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  return ((sam_serialin8(priv, SAM_USART_INTFLAG_OFFSET) & USART_INT_RXC)
          != 0);
}

/****************************************************************************
 * Name: sam_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void sam_send(struct uart_dev_s *dev, int ch)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      sam_portwrite(priv->rs485_dir_gpio, priv->rs485_dir_polarity);
    }
#endif

  sam_serialout16(priv, SAM_USART_DATA_OFFSET, (uint16_t)ch);
}

/****************************************************************************
 * Name: sam_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void sam_txint(struct uart_dev_s *dev, bool enable)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      sam_serialout8(priv, SAM_USART_INTENSET_OFFSET, USART_TX_INTS);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);

#endif
    }
  else
    {
      /* Disable the TX interrupt. Only disable DRE, TXC will disable
       * itself!
       */

      sam_serialout8(priv, SAM_USART_INTENCLR_OFFSET, USART_INT_DRE);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool sam_txempty(struct uart_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev->priv;
  return ((sam_serialin8(priv, SAM_USART_INTFLAG_OFFSET) & USART_INT_DRE)
          != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before sam_serialinit.
 *
 *   NOTE: On this platform arm_earlyserialinit() does not really do
 *   anything of consequence and probably could be eliminated with little
 *   effort.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* Disable all USARTS */

  sam_disableallints(TTYS0_DEV.priv);
#ifdef TTYS1_DEV
  sam_disableallints(TTYS1_DEV.priv);
#endif
#ifdef TTYS2_DEV
  sam_disableallints(TTYS2_DEV.priv);
#endif
#ifdef TTYS3_DEV
  sam_disableallints(TTYS3_DEV.priv);
#endif
#ifdef TTYS4_DEV
  sam_disableallints(TTYS4_DEV.priv);
#endif
#ifdef TTYS5_DEV
  sam_disableallints(TTYS5_DEV.priv);
#endif

#ifdef HAVE_SERIAL_CONSOLE
  /* Mark the serial console (if any) */

  CONSOLE_DEV.isconsole = true;
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that sam_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all USARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
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

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  irqstate_t flags;

  /* All interrupts must be disabled to prevent re-entrancy and to prevent
   * interrupts from firing in the serial driver code.
   */

  flags = enter_critical_section();

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      sam_lowputc('\r');
    }

  sam_lowputc(ch);
  leave_critical_section(flags);
#endif
  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      sam_lowputc('\r');
    }

  sam_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
#endif /* SAMD2L2_HAVE_USART */
