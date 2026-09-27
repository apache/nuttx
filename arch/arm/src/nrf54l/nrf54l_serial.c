/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_serial.c
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

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "nrf54l_config.h"
#include "nrf54l_lowputc.h"
#include "nrf54l_serial.h"
#include "hardware/nrf54l_uarte.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of UART instances */

#define NRF54L_NUART 7

#if defined(HAVE_UART_DEVICE) && defined(USE_SERIALDRIVER)

/* The console is always ttyS0. */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_uart0port
#  define TTYS0_DEV   g_uart0port
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_uart1port
#  define TTYS0_DEV   g_uart1port
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_uart2port
#  define TTYS0_DEV   g_uart2port
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_uart3port
#  define TTYS0_DEV   g_uart3port
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_uart4port
#  define TTYS0_DEV   g_uart4port
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_uart5port
#  define TTYS0_DEV   g_uart5port
#elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#  define CONSOLE_DEV g_uart6port
#  define TTYS0_DEV   g_uart6port
#endif

#define RX_INTERRUPTS (UARTE_INT_DMA_RX_END | UARTE_INT_ERROR | \
                       UARTE_INT_DMA_RX_BUSERROR | UARTE_INT_RXTO)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* State of one UART device */

struct nrf54l_dev_s
{
  uintptr_t     uartbase;       /* Base address of UART registers */
  uint16_t      irq;            /* IRQ associated with this UART */
  uint8_t       rxdma;          /* Receive DMA buffer */
  uint8_t       txdma;          /* Transmit DMA buffer */
  uint8_t       rxchar;         /* Completed receive byte */
  uint32_t      rxstatus;       /* Receive error status */
  bool          rxvalid;        /* Receive byte available */
  bool          rxenabled;      /* Upper-half reception enabled */
  bool          rxrecover;      /* Receiver stopping after a DMA error */
  bool          rxactive;       /* Receiver started */
  bool          txenabled;      /* Upper-half transmission enabled */
  volatile bool txbusy;         /* DMA transmitter owns txdma */

  /* UART configuration */

  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  nrf54l_setup(struct uart_dev_s *dev);
static void nrf54l_shutdown(struct uart_dev_s *dev);
static int  nrf54l_attach(struct uart_dev_s *dev);
static void nrf54l_detach(struct uart_dev_s *dev);
static int  nrf54l_interrupt(int irq, void *context, void *arg);
static int  nrf54l_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  nrf54l_receive(struct uart_dev_s *dev, unsigned int *status);
static void nrf54l_rxint(struct uart_dev_s *dev, bool enable);
static bool nrf54l_rxavailable(struct uart_dev_s *dev);
static void nrf54l_send(struct uart_dev_s *dev, int ch);
static void nrf54l_txint(struct uart_dev_s *dev, bool enable);
static bool nrf54l_txready(struct uart_dev_s *dev);
static bool nrf54l_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = nrf54l_setup,
  .shutdown       = nrf54l_shutdown,
  .attach         = nrf54l_attach,
  .detach         = nrf54l_detach,
  .ioctl          = nrf54l_ioctl,
  .receive        = nrf54l_receive,
  .rxint          = nrf54l_rxint,
  .rxavailable    = nrf54l_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = nrf54l_send,
  .txint          = nrf54l_txint,
  .txready        = nrf54l_txready,
  .txempty        = nrf54l_txempty,
};

/* I/O buffers */

#ifdef CONFIG_NRF54L_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_NRF54L_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_NRF54L_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_NRF54L_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif

#ifdef CONFIG_NRF54L_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif

#ifdef CONFIG_NRF54L_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
#endif

#ifdef CONFIG_NRF54L_UART6
static char g_uart6rxbuffer[CONFIG_UART6_RXBUFSIZE];
static char g_uart6txbuffer[CONFIG_UART6_TXBUFSIZE];
#endif

/* This describes the state of the NRF54L UART0 port. */

#ifdef CONFIG_NRF54L_UART0
static struct nrf54l_dev_s g_uart0priv =
{
  .uartbase = NRF54L_UART0_BASE,
  .irq      = NRF54L_IRQ_SERIAL20,
  .config   =
  {
    .baud      = CONFIG_UART0_BAUD,
    .parity    = CONFIG_UART0_PARITY,
    .bits      = CONFIG_UART0_BITS,
    .stopbits2 = CONFIG_UART0_2STOP,
    .txpin     = BOARD_UART0_TX_PIN,
    .rxpin     = BOARD_UART0_RX_PIN,
  },
};

static struct uart_dev_s g_uart0port =
{
  .recv =
  {
    .size   = sizeof(g_uart0rxbuffer),
    .buffer = g_uart0rxbuffer,
  },
  .xmit =
  {
    .size   = sizeof(g_uart0txbuffer),
    .buffer = g_uart0txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart0priv,
};

#endif

/* This describes the state of the NRF54L UART1 port. */

#ifdef CONFIG_NRF54L_UART1
static struct nrf54l_dev_s g_uart1priv =
{
  .uartbase = NRF54L_UART1_BASE,
  .irq      = NRF54L_IRQ_SERIAL21,
  .config   =
  {
    .baud      = CONFIG_UART1_BAUD,
    .parity    = CONFIG_UART1_PARITY,
    .bits      = CONFIG_UART1_BITS,
    .stopbits2 = CONFIG_UART1_2STOP,
    .txpin     = BOARD_UART1_TX_PIN,
    .rxpin     = BOARD_UART1_RX_PIN,
  },
};

static struct uart_dev_s g_uart1port =
{
  .recv =
  {
    .size   = sizeof(g_uart1rxbuffer),
    .buffer = g_uart1rxbuffer,
  },
  .xmit =
  {
    .size   = sizeof(g_uart1txbuffer),
    .buffer = g_uart1txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart1priv,
};

#endif

/* This describes the state of the NRF54L UART2 port. */

#ifdef CONFIG_NRF54L_UART2
static struct nrf54l_dev_s g_uart2priv =
{
  .uartbase = NRF54L_UART2_BASE,
  .irq      = NRF54L_IRQ_SERIAL22,
  .config   =
  {
    .baud      = CONFIG_UART2_BAUD,
    .parity    = CONFIG_UART2_PARITY,
    .bits      = CONFIG_UART2_BITS,
    .stopbits2 = CONFIG_UART2_2STOP,
    .txpin     = BOARD_UART2_TX_PIN,
    .rxpin     = BOARD_UART2_RX_PIN,
  },
};

static struct uart_dev_s g_uart2port =
{
  .recv =
  {
    .size   = sizeof(g_uart2rxbuffer),
    .buffer = g_uart2rxbuffer,
  },
  .xmit =
  {
    .size   = sizeof(g_uart2txbuffer),
    .buffer = g_uart2txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart2priv,
};

#endif

/* This describes the state of the NRF54L UART3 port. */

#ifdef CONFIG_NRF54L_UART3
static struct nrf54l_dev_s g_uart3priv =
{
  .uartbase = NRF54L_UART3_BASE,
  .irq      = NRF54L_IRQ_SERIAL30,
  .config   =
  {
    .baud      = CONFIG_UART3_BAUD,
    .parity    = CONFIG_UART3_PARITY,
    .bits      = CONFIG_UART3_BITS,
    .stopbits2 = CONFIG_UART3_2STOP,
    .txpin     = BOARD_UART3_TX_PIN,
    .rxpin     = BOARD_UART3_RX_PIN,
  },
};

static struct uart_dev_s g_uart3port =
{
  .recv =
  {
    .size   = sizeof(g_uart3rxbuffer),
    .buffer = g_uart3rxbuffer,
  },
  .xmit =
  {
    .size   = sizeof(g_uart3txbuffer),
    .buffer = g_uart3txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart3priv,
};

#endif

/* This describes the state of the NRF54L UART4 port. */

#ifdef CONFIG_NRF54L_UART4
static struct nrf54l_dev_s g_uart4priv =
{
  .uartbase = NRF54L_UART4_BASE,
  .irq      = NRF54L_IRQ_SERIAL00,
  .config   =
  {
    .baud      = CONFIG_UART4_BAUD,
    .parity    = CONFIG_UART4_PARITY,
    .bits      = CONFIG_UART4_BITS,
    .stopbits2 = CONFIG_UART4_2STOP,
    .txpin     = BOARD_UART4_TX_PIN,
    .rxpin     = BOARD_UART4_RX_PIN,
  },
};

static struct uart_dev_s g_uart4port =
{
  .recv =
  {
    .size   = sizeof(g_uart4rxbuffer),
    .buffer = g_uart4rxbuffer,
  },
  .xmit =
  {
    .size   = sizeof(g_uart4txbuffer),
    .buffer = g_uart4txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart4priv,
};

#endif

/* This describes the state of the NRF54L UART5 port. */

#ifdef CONFIG_NRF54L_UART5
static struct nrf54l_dev_s g_uart5priv =
{
  .uartbase = NRF54L_UART5_BASE,
  .irq      = NRF54L_IRQ_SERIAL23,
  .config   =
  {
    .baud      = CONFIG_UART5_BAUD,
    .parity    = CONFIG_UART5_PARITY,
    .bits      = CONFIG_UART5_BITS,
    .stopbits2 = CONFIG_UART5_2STOP,
    .txpin     = BOARD_UART5_TX_PIN,
    .rxpin     = BOARD_UART5_RX_PIN,
  },
};

static struct uart_dev_s g_uart5port =
{
  .recv =
  {
    .size   = sizeof(g_uart5rxbuffer),
    .buffer = g_uart5rxbuffer,
  },
  .xmit =
  {
    .size   = sizeof(g_uart5txbuffer),
    .buffer = g_uart5txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart5priv,
};

#endif

/* This describes the state of the NRF54L UART6 port. */

#ifdef CONFIG_NRF54L_UART6
static struct nrf54l_dev_s g_uart6priv =
{
  .uartbase = NRF54L_UART6_BASE,
  .irq      = NRF54L_IRQ_SERIAL24,
  .config   =
  {
    .baud      = CONFIG_UART6_BAUD,
    .parity    = CONFIG_UART6_PARITY,
    .bits      = CONFIG_UART6_BITS,
    .stopbits2 = CONFIG_UART6_2STOP,
    .txpin     = BOARD_UART6_TX_PIN,
    .rxpin     = BOARD_UART6_RX_PIN,
  },
};

static struct uart_dev_s g_uart6port =
{
  .recv =
  {
    .size   = sizeof(g_uart6rxbuffer),
    .buffer = g_uart6rxbuffer,
  },
  .xmit =
  {
    .size   = sizeof(g_uart6txbuffer),
    .buffer = g_uart6txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart6priv,
};

#endif

/* This table lets us iterate over the configured UARTs */

static struct uart_dev_s * const g_uart_devs[NRF54L_NUART] =
{
#ifdef CONFIG_NRF54L_UART0
  [0] = &g_uart0port,
#endif
#ifdef CONFIG_NRF54L_UART1
  [1] = &g_uart1port,
#endif
#ifdef CONFIG_NRF54L_UART2
  [2] = &g_uart2port,
#endif
#ifdef CONFIG_NRF54L_UART3
  [3] = &g_uart3port,
#endif
#ifdef CONFIG_NRF54L_UART4
  [4] = &g_uart4port,
#endif
#ifdef CONFIG_NRF54L_UART5
  [5] = &g_uart5port,
#endif
#ifdef CONFIG_NRF54L_UART6
  [6] = &g_uart6port,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_uart_putreg
 *
 * Description:
 *   Write a UART register.
 *
 ****************************************************************************/

static inline void nrf54l_uart_putreg(struct nrf54l_dev_s *priv,
                                      uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: nrf54l_uart_getreg
 *
 * Description:
 *   Read a UART register.
 *
 ****************************************************************************/

static inline uint32_t nrf54l_uart_getreg(struct nrf54l_dev_s *priv,
                                          uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: nrf54l_uart_clrevent
 *
 * Description:
 *   Clear a UARTE event and read it back before returning.
 *
 ****************************************************************************/

static void nrf54l_uart_clrevent(struct nrf54l_dev_s *priv, uint32_t event)
{
  nrf54l_uart_putreg(priv, event, 0);
  (void)nrf54l_uart_getreg(priv, event);
}

/****************************************************************************
 * Name: nrf54l_rxstart
 *
 * Description:
 *   Start reception and queue the first one-byte DMA buffer.
 *
 ****************************************************************************/

static void nrf54l_rxstart(struct nrf54l_dev_s *priv)
{
  priv->rxactive = true;
  priv->rxvalid = false;
  nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_DMA_RX_END_OFFSET);
  nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_DMA_RX_READY_OFFSET);
  nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_RXTO_OFFSET);
  nrf54l_uart_putreg(priv, NRF54L_UARTE_DMA_RX_PTR_OFFSET,
                     (uintptr_t)&priv->rxdma);
  nrf54l_uart_putreg(priv, NRF54L_UARTE_DMA_RX_MAXCNT_OFFSET, 1);
  nrf54l_uart_putreg(priv, NRF54L_UARTE_SHORTS_OFFSET,
                     UARTE_SHORTS_DMA_TX_END_STOP);
  UP_DMB();
  nrf54l_uart_putreg(priv, NRF54L_UARTE_TASKS_STARTRX_OFFSET, 1);
}

/****************************************************************************
 * Name: nrf54l_txstart
 *
 * Description:
 *   Start a one-byte DMA transfer. The caller must serialize access.
 *
 ****************************************************************************/

static void nrf54l_txstart(struct nrf54l_dev_s *priv, int ch)
{
  priv->txdma = ch;
  priv->txbusy = true;
  nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_DMA_TX_END_OFFSET);
  nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_TXSTOPPED_OFFSET);
  nrf54l_uart_putreg(priv, NRF54L_UARTE_DMA_TX_PTR_OFFSET,
                     (uintptr_t)&priv->txdma);
  nrf54l_uart_putreg(priv, NRF54L_UARTE_DMA_TX_MAXCNT_OFFSET, 1);
  UP_DMB();
  nrf54l_uart_putreg(priv, NRF54L_UARTE_TASKS_STARTTX_OFFSET, 1);
}

/****************************************************************************
 * Name: nrf54l_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int nrf54l_setup(struct uart_dev_s *dev)
{
  struct nrf54l_dev_s *priv = dev->priv;

  return nrf54l_usart_configure(priv->uartbase, &priv->config);
}

/****************************************************************************
 * Name: nrf54l_shutdown
 *
 * Description:
 *   Disable the UART. This method is not called for the serial console.
 *
 ****************************************************************************/

static void nrf54l_shutdown(struct uart_dev_s *dev)
{
  struct nrf54l_dev_s *priv = dev->priv;

  nrf54l_usart_disable(priv->uartbase, &priv->config);
}

/****************************************************************************
 * Name: nrf54l_attach
 *
 * Description:
 *   Configure the UART to operate in interrupt driven mode. This method
 *   is called when the serial port is opened, normally just after setup.
 *   DMA completion interrupts release buffers even when delivery to the
 *   upper half is disabled by the rxint() and txint() methods.
 *
 ****************************************************************************/

static int nrf54l_attach(struct uart_dev_s *dev)
{
  struct nrf54l_dev_s *priv = (struct nrf54l_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, nrf54l_interrupt, dev);
  if (ret == OK)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      nrf54l_uart_putreg(priv, NRF54L_UARTE_INTENSET_OFFSET, RX_INTERRUPTS |
                         UARTE_INT_TXSTOPPED | UARTE_INT_DMA_TX_BUSERROR);
      up_enable_irq(priv->irq);
      nrf54l_rxstart(priv);
#endif
    }

  return ret;
}

/****************************************************************************
 * Name: nrf54l_detach
 *
 * Description:
 *   Detach UART interrupts. This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void nrf54l_detach(struct uart_dev_s *dev)
{
  struct nrf54l_dev_s *priv = (struct nrf54l_dev_s *)dev->priv;

  /* Stop DMA before detaching the interrupt handler */

  nrf54l_uart_putreg(priv, NRF54L_UARTE_INTENCLR_OFFSET, UINT32_MAX);
  nrf54l_uart_putreg(priv, NRF54L_UARTE_SHORTS_OFFSET,
                     UARTE_SHORTS_DMA_TX_END_STOP);
  if (priv->rxactive)
    {
      /* Wait until the DMA buffer is released before a subsequent attach. */

      nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_RXTO_OFFSET);
      nrf54l_uart_putreg(priv, NRF54L_UARTE_TASKS_STOPRX_OFFSET, 1);
      while (!nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_RXTO_OFFSET))
        {
        }

      priv->rxactive = false;
      priv->rxrecover = false;
    }

  if (priv->txbusy)
    {
      while (!nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_TXSTOPPED_OFFSET))
        {
        }

      priv->txbusy = false;
    }

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: nrf54l_interrupt
 *
 * Description:
 *   Handle receive buffers, transfer completion and DMA errors.
 *
 ****************************************************************************/

static int nrf54l_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = arg;
  struct nrf54l_dev_s *priv = dev->priv;

  if (nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_ERROR_OFFSET))
    {
      nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_ERROR_OFFSET);
      priv->rxstatus |=
        nrf54l_uart_getreg(priv, NRF54L_UARTE_ERRORSRC_OFFSET);
      nrf54l_uart_putreg(priv, NRF54L_UARTE_ERRORSRC_OFFSET, priv->rxstatus);
    }

  if (nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_DMA_RX_BUSERROR_OFFSET))
    {
      nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_DMA_RX_BUSERROR_OFFSET);
      priv->rxrecover = true;
      nrf54l_uart_putreg(priv, NRF54L_UARTE_SHORTS_OFFSET,
                         UARTE_SHORTS_DMA_TX_END_STOP);
      nrf54l_uart_putreg(priv, NRF54L_UARTE_TASKS_STOPRX_OFFSET, 1);
    }

  if (priv->rxrecover)
    {
      if (nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_RXTO_OFFSET))
        {
          priv->rxrecover = false;
          nrf54l_rxstart(priv);
        }
    }
  else
    {
      /* Copy the completed byte before restarting DMA. Automatic END-to-
       * START would let DMA overwrite a buffer before the interrupt handler
       * consumes it. The UART FIFO holds incoming bytes between transfers.
       */

      if (nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_DMA_RX_END_OFFSET))
        {
          nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_DMA_RX_END_OFFSET);
          UP_DMB();
          priv->rxchar = priv->rxdma;
          priv->rxvalid = true;
          UP_DMB();
          nrf54l_uart_putreg(priv, NRF54L_UARTE_TASKS_STARTRX_OFFSET, 1);
        }

      if (priv->rxvalid && priv->rxenabled)
        {
          uart_recvchars(dev);
        }

      priv->rxvalid = false;
      nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_RXTO_OFFSET);
    }

  if (nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_DMA_TX_BUSERROR_OFFSET))
    {
      nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_DMA_TX_BUSERROR_OFFSET);
      nrf54l_uart_putreg(priv, NRF54L_UARTE_TASKS_STOPTX_OFFSET, 1);
    }

  if (nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_TXSTOPPED_OFFSET))
    {
      nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_TXSTOPPED_OFFSET);
      priv->txbusy = false;
      if (priv->txenabled)
        {
          uart_xmitchars(dev);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nrf54l_set_format
 *
 * Description:
 *   Set the serial line format and speed with DMA stopped.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TERMIOS
static int nrf54l_set_format(struct uart_dev_s *dev,
                             const struct uart_config_s *config)
{
  struct nrf54l_dev_s *priv = dev->priv;
  irqstate_t flags;
  uint32_t enabled;
  int ret;

  flags = enter_critical_section();

  /* Do not change the format of a byte still being transmitted or
   * interrupt DMA error recovery.
   */

  if (priv->txbusy || priv->rxrecover)
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  if (priv->rxactive)
    {
      nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_RXTO_OFFSET);
      nrf54l_uart_putreg(priv, NRF54L_UARTE_TASKS_STOPRX_OFFSET, 1);
      while (!nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_RXTO_OFFSET))
        {
        }

      /* STOP also generates END for an empty buffer. Deliver a completed
       * byte before restarting DMA, but never deliver an empty buffer.
       */

      if (nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_ERROR_OFFSET))
        {
          nrf54l_uart_clrevent(priv, NRF54L_UARTE_EVENTS_ERROR_OFFSET);
          priv->rxstatus |=
            nrf54l_uart_getreg(priv, NRF54L_UARTE_ERRORSRC_OFFSET);
          nrf54l_uart_putreg(priv, NRF54L_UARTE_ERRORSRC_OFFSET,
                             priv->rxstatus);
        }

      if (nrf54l_uart_getreg(priv, NRF54L_UARTE_DMA_RX_AMOUNT_OFFSET) != 0 &&
          !nrf54l_uart_getreg(priv,
                              NRF54L_UARTE_EVENTS_DMA_RX_BUSERROR_OFFSET))
        {
          UP_DMB();
          priv->rxchar = priv->rxdma;
          priv->rxvalid = true;
          if (priv->rxenabled)
            {
              uart_recvchars(dev);
            }
        }
    }

  enabled = nrf54l_uart_getreg(priv, NRF54L_UARTE_ENABLE_OFFSET);
  nrf54l_uart_putreg(priv, NRF54L_UARTE_ENABLE_OFFSET, 0);
  ret = nrf54l_usart_setformat(priv->uartbase, config);
  if (ret == OK)
    {
      priv->config = *config;
    }

  nrf54l_uart_putreg(priv, NRF54L_UARTE_ENABLE_OFFSET, enabled);
  if (priv->rxactive)
    {
      nrf54l_rxstart(priv);
    }

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: nrf54l_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int nrf54l_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode         *inode  = filep->f_inode;
  struct uart_dev_s    *dev    = inode->i_private;
  struct nrf54l_dev_s  *priv   = dev->priv;
  struct uart_config_s config;
  irqstate_t flags;
#endif
  int ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          flags = enter_critical_section();
          config = priv->config;
          leave_critical_section(flags);

          termiosp->c_cflag = ((config.parity != 0) ? PARENB : 0)
                              | ((config.parity == 1) ? PARODD : 0)
                              | ((config.stopbits2) ? CSTOPB : 0);

          switch (config.bits)
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
                termiosp->c_cflag |= CS8;
                break;
            }

          cfsetispeed(termiosp, config.baud);
          break;
        }

      case TCSETS:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* Hardware flow control is not supported. */

          if ((termiosp->c_cflag & (CCTS_OFLOW | CRTS_IFLOW)) != 0)
            {
              ret = -EINVAL;
              break;
            }

          flags = enter_critical_section();
          config = priv->config;
          leave_critical_section(flags);

          switch (termiosp->c_cflag & CSIZE)
            {
              case CS5:
                config.bits = 5;
                break;

              case CS6:
                config.bits = 6;
                break;

              case CS7:
                config.bits = 7;
                break;

              case CS8:
                config.bits = 8;
                break;

              default:
                return -EINVAL;
            }

          /* Parity */

          if (termiosp->c_cflag & PARENB)
            {
              config.parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              config.parity = 0;
            }

          /* Stop bits */

          config.stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;

          /* Note that only cfgetispeed is used because we have knowledge
           * that only one speed is supported.
           */

          config.baud = cfgetispeed(termiosp);

          /* Effect the changes */

          ret = nrf54l_set_format(dev, &config);
          break;
        }
#endif

      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrf54l_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int nrf54l_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct nrf54l_dev_s *priv = dev->priv;

  if (status)
    {
      *status = priv->rxstatus;
    }

  priv->rxstatus = 0;
  priv->rxvalid = false;
  return priv->rxchar;
}

/****************************************************************************
 * Name: nrf54l_rxint
 *
 * Description:
 *   Enable or disable delivery of received characters to the upper half.
 *
 ****************************************************************************/

static void nrf54l_rxint(struct uart_dev_s *dev, bool enable)
{
  struct nrf54l_dev_s *priv = dev->priv;

#ifdef CONFIG_SUPPRESS_SERIAL_INTS
  priv->rxenabled = false;
#else
  priv->rxenabled = enable;
#endif
}

/****************************************************************************
 * Name: nrf54l_rxavailable
 *
 * Description:
 *   Report whether a received character is waiting for the upper half.
 *
 ****************************************************************************/

static bool nrf54l_rxavailable(struct uart_dev_s *dev)
{
  struct nrf54l_dev_s *priv = dev->priv;

  return priv->rxvalid;
}

/****************************************************************************
 * Name: nrf54l_send
 *
 * Description:
 *   Start transmitting the character supplied by the upper half.
 *
 ****************************************************************************/

static void nrf54l_send(struct uart_dev_s *dev, int ch)
{
  struct nrf54l_dev_s *priv = dev->priv;
  irqstate_t flags = enter_critical_section();

  nrf54l_txstart(priv, ch);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf54l_txint
 *
 * Description:
 *   Enable or disable transmission from the upper-half transmit buffer.
 *
 ****************************************************************************/

static void nrf54l_txint(struct uart_dev_s *dev, bool enable)
{
  struct nrf54l_dev_s *priv = dev->priv;
  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_SUPPRESS_SERIAL_INTS
  enable = false;
#endif

  priv->txenabled = enable;
  if (enable)
    {
      uart_xmitchars(dev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf54l_txready
 *
 * Description:
 *   Report whether the transmitter can accept another character.
 *
 ****************************************************************************/

static bool nrf54l_txready(struct uart_dev_s *dev)
{
  struct nrf54l_dev_s *priv = dev->priv;

  return !priv->txbusy;
}

/****************************************************************************
 * Name: nrf54l_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool nrf54l_txempty(struct uart_dev_s *dev)
{
  struct nrf54l_dev_s *priv = dev->priv;

  return !priv->txbusy;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before arm_serialinit.  NOTE: This function depends on GPIO pin
 *   configuration performed in nrf54l_lowsetup() and main clock
 *   initialization performed in nrf54l_clockconfig().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void nrf54l_earlyserialinit(void)
{
  /* Configure whichever one is the console */

#ifdef HAVE_UART_CONSOLE
  CONSOLE_DEV.isconsole = true;
  nrf54l_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that nrf54l_earlyserialinit was called previously.
 *
 * Input Parameters:
 *   None
 *
 * Returns Value:
 *   None
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  unsigned minor = 0;
  unsigned i     = 0;
  char devname[16];

#ifdef HAVE_UART_CONSOLE
  /* Register the serial console */

  CONSOLE_DEV.isconsole = true;
  uart_register("/dev/console", &CONSOLE_DEV);
  uart_register("/dev/ttyS0", &TTYS0_DEV);
  minor = 1;
#endif

  /* Register all remaining UARTs */

  strlcpy(devname, "/dev/ttySx", sizeof(devname));

  for (i = 0; i < NRF54L_NUART; i++)
    {
      /* Don't create a device for non-configured ports. */

      if (g_uart_devs[i] == NULL)
        {
          continue;
        }

      /* Don't create a device for the console - we did that above */

      if (g_uart_devs[i]->isconsole)
        {
          continue;
        }

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + minor++;
      uart_register(devname, g_uart_devs[i]);
    }
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes.
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_UART_CONSOLE
  struct nrf54l_dev_s *priv = CONSOLE_DEV.priv;
  irqstate_t flags = enter_critical_section();

  /* Serialize polled debug output with the interrupt-driven transmitter.
   * TXSTOPPED, rather than DMA END, guarantees the pin is idle before reuse.
   */

  if (priv->txbusy)
    {
      while (!nrf54l_uart_getreg(priv, NRF54L_UARTE_EVENTS_TXSTOPPED_OFFSET))
        {
        }
    }

  arm_lowputc(ch);
  priv->txbusy = false;
  leave_critical_section(flags);
#endif
}

#else /* HAVE_UART_DEVICE && USE_SERIALDRIVER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes.
 *
 ****************************************************************************/

void up_putc(int ch)
{
  arm_lowputc(ch);
}

#endif /* HAVE_UART_DEVICE && USE_SERIALDRIVER */
